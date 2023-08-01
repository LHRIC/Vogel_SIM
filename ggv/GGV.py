import numpy as np
import math
import utilities.MF52 as MF52
import matplotlib.pyplot as plt
from csaps import csaps

class GGV:
    def __init__(self, AERO, DYN, PTN, gear_tot, v_max):
        self.AERO = AERO
        self.DYN = DYN
        self.PTN = PTN
        self.gear_tot = gear_tot
        self.v_max = v_max

        self.velocity_range = np.arange(4, math.floor(self.v_max) + 1, 1)
        self.radii_range = np.arange(3.5, 37.5, 1.5)
        self.curr_gear = 1
        self.shift_count = 1

        self._MF52 = MF52()

    def calc_grip_lim_max_accel(self, v):
        downforce = self.AERO.Cl * v**2 # Downforce: Newtons
        #dxr
        rear_sus_drop = downforce * (1 - self.AERO.CoP) / 2 / self.DYN.ride_rate_r
        #IA_0r
        IA_0r = self.DYN.static_camber_r - rear_sus_drop * self.DYN.camber_gain_r

        A_x_diff = 1
        Ax = 0
        
        while A_x_diff > 0:
            Ax += 0.01
            pitch = -Ax * self.DYN.pitch_grad
            wr = (self.DYN.total_weight_r + downforce*(1-self.AERO.CoP)) / 2
            wr += Ax * self.DYN.cg_height * self.DYN.total_weight / self.DYN.wheelbase / 2
            IA_r = self.DYN.wheelbase * math.sin(pitch)/2*self.DYN.camber_gain_r + IA_0r

            fxr = []
            for sl in np.arange(0, 1.01, 0.01):
                Fx = self._MF52.Fx(-wr, -IA_r, sl)*self.DYN.friction_scaling_x
                fxr.append(Fx)
            FXR = max(fxr)
            FX = abs(2*FXR)
            AX = FX/self.DYN.total_weight # THIS IS IN G'S, SEE ABOVE
            A_x_diff = AX-Ax
        
        return AX
    
    def calc_power_lim_max_accel(self, v):
        gear_idx = 0
        rpm = self.PTN.shiftpoint

        # Short little accel simulation, determine what gear the car will be in
        # if it had to accelerate to this velocity
        while rpm >= self.PTN.shiftpoint:
            total_red = self.PTN.gear_ratios[gear_idx] * self.PTN.final_drive * self.PTN.primary_reduction
            gear_idx += 1
            rpm = v*total_red/self.DYN.tire_radius*60/(2*math.pi)

        # Determine torque at the crankshaft
        crankshaft_torque = np.interp(rpm, self.PTN.rpm_range, self.PTN.torque_curve)
        wheel_torque = crankshaft_torque * total_red * self.PTN.drivetrain_losses
        Fx = wheel_torque / self.DYN.tire_radius

        return (Fx, gear_idx)

        

    def generate(self):
        power_lim_a = []
        grip_lim_a = []
        accel_cap = []
        for v in self.velocity_range:
            Ax_r = self.calc_grip_lim_max_accel(v)
            grip_lim_a.append(Ax_r)

            FX_r, gear_idx = self.calc_power_lim_max_accel(v)
            FX_r -= self.AERO.Cl * v**2 # Downforce: Newtons
            AX_r = FX_r / self.DYN.total_weight 
            power_lim_a.append(AX_r)

            accel_cap.append(min(Ax_r, AX_r))
        
        xi = np.linspace(self.velocity_range[0], self.velocity_range[-1], 150)
        yi = csaps(self.velocity_range, accel_cap, xi, smooth=0.9)
            
        fig, ax = plt.subplots()
        ax.plot(self.velocity_range, accel_cap, 'o')
        ax.plot(xi, yi)
        ax.plot(self.velocity_range, grip_lim_a)
        plt.show()



            
            