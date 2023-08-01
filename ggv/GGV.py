import numpy as np
import math
import utilities.MF52 as MF52

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
        Ax_r = []
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
        
        Ax_r.append(AX)
        return Ax_r
    
    
        

    def generate(self):

        for v in self.velocity_range:
            Ax_r = self.calc_grip_lim_max_accel(v)
            
            