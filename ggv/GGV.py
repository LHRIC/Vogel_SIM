import numpy as np
from numpy.polynomial import Polynomial
import math
import utilities.MF52 as MF52
import matplotlib.pyplot as plt
from csaps import csaps
from scipy.optimize import least_squares
import statistics
import pickle


class GGV:
    def __init__(self, AERO, DYN, PTN, gear_tot, v_max):
        self.AERO = AERO
        self.DYN = DYN
        self.PTN = PTN
        self.gear_tot = gear_tot
        self.v_max = v_max

        self.velocity_range = np.arange(4, math.floor(self.v_max) + 1, 1)
        self.radii_range = np.arange(3.5, 37.5, 1.5)
        print(len(self.velocity_range))
        print(len(self.radii_range))
        exit()
        self.curr_gear = 1
        self.shift_count = 1

        self._MF52 = MF52()

    def calc_grip_lim_max_accel(self, v):
        downforce = self.AERO.Cl * v**2  # Downforce: Newtons
        # dxr
        rear_sus_drop = downforce * (1 - self.AERO.CoP) / 2 / self.DYN.ride_rate_r
        # IA_0r
        IA_0r = self.DYN.static_camber_r - rear_sus_drop * self.DYN.camber_gain_r

        A_x_diff = 1
        Ax = 0

        while A_x_diff > 0:
            Ax += 0.01
            pitch = -Ax * self.DYN.pitch_grad
            wr = (self.DYN.total_weight_r + downforce * (1 - self.AERO.CoP)) / 2
            wr += (
                Ax * self.DYN.cg_height * self.DYN.total_weight / self.DYN.wheelbase / 2
            )
            IA_r = (
                self.DYN.wheelbase * math.sin(pitch) / 2 * self.DYN.camber_gain_r
                + IA_0r
            )

            fxr = []
            for sl in np.arange(0, 1.01, 0.01):
                Fx = self._MF52.Fx(-wr, -IA_r, sl) * self.DYN.friction_scaling_x
                fxr.append(Fx)
            FXR = max(fxr)
            FX = abs(2 * FXR)
            AX = FX / self.DYN.total_weight  # THIS IS IN G'S, SEE ABOVE
            A_x_diff = AX - Ax

        return AX

    def calc_power_lim_max_accel(self, v):
        gear_idx = 0
        rpm = self.PTN.shiftpoint

        # Short little accel simulation, determine what gear the car will be in
        # if it had to accelerate to this velocity
        while rpm >= self.PTN.shiftpoint:
            total_red = (
                self.PTN.gear_ratios[gear_idx]
                * self.PTN.final_drive
                * self.PTN.primary_reduction
            )
            gear_idx += 1
            rpm = v * total_red / self.DYN.tire_radius * 60 / (2 * math.pi)

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
            FX_r -= self.AERO.Cl * v**2  # Downforce: Newtons
            AX_r = FX_r / self.DYN.total_weight
            power_lim_a.append(AX_r)

            accel_cap.append(min(Ax_r, AX_r))

        self.velocity_fit_x = np.linspace(
            self.velocity_range[0], self.velocity_range[-1], 150
        )
        self.accel_cap_fit_y = csaps(
            self.velocity_range, accel_cap, self.velocity_fit_x, smooth=0.9
        )
        self.grip_cap_fit_y = csaps(
            self.velocity_range, grip_lim_a, self.velocity_fit_x, smooth=0.9
        )

        """ 
        fig, ax = plt.subplots()
        ax.plot(self.velocity_range, accel_cap, 'o')
        ax.plot(xi, yi)
        ax.plot(self.velocity_range, grip_lim_a)
        plt.show()
        

        fig, ax = plt.subplots()
        ax.plot(self.velocity_range, power_lim_a, "o")
        ax.plot(velocity_fit_x, accel_cap_fit_y)
        ax.plot(velocity_fit_x, grip_cap_fit_y)
        ax.plot(self.velocity_range, grip_lim_a, "x")
        plt.show()
        """

        AYP = 0.5
        lateral_g = []
        delta_l = []
        beta_l = []
        AYP_l = []
        for R in self.radii_range:
            self.a = self.DYN.wheelbase * (1 - self.DYN.weight_dist_f)
            self.b = self.DYN.wheelbase * self.DYN.weight_dist_f
            self.R = R
            # update speed and downforce
            V = math.sqrt(R * 9.81 * AYP)
            LF = self.AERO.Cl * V**2

            # Update suspension travel
            dxf = LF * self.AERO.CoP / 2 / self.DYN.ride_rate_f
            dxr = LF * (1 - self.AERO.CoP) / 2 / self.DYN.ride_rate_r

            # Get static camber from suspension heave
            self.IA_0f = self.DYN.static_camber_f - dxf * self.DYN.camber_gain_f
            self.IA_0r = self.DYN.static_camber_r - dxr * self.DYN.camber_gain_r

            # Get loads on each wheel
            self.wf = (self.DYN.total_weight_f + LF * self.AERO.CoP) / 2
            self.wr = (self.DYN.total_weight_r + LF * (1 - self.AERO.CoP)) / 2

            # Guess initial ackermann
            delta = self.DYN.wheelbase / R

            # Assume sideslip starts at 0
            beta = 0
            # a, b, R, wf, wr, IA_0f, IA_0r, 0, velocity_fit_x, grip_cap_fit_y
            x0 = [delta, beta, AYP]
            lb = [0.01, -.3, .5]
            ub = [1, .3, 2]

            x = least_squares(self.vogel, x0, bounds=(lb, ub))

            delta = x.x[0]
            beta = x.x[1]
            AYP = x.x[2]

            #delta_l.append(delta)
            #beta_l.append(beta)
            #AYP_l.append(AYP)
            
            V = math.sqrt(self.R * 9.81 * AYP)
            A_y = V**2 / self.R / 9.81  # (g's)

            lateral_g.append(A_y)
            
        
        lateral_g = np.array(lateral_g)
        velocity_y = lateral_g*9.81*self.radii_range
        velocity_y = np.sqrt(velocity_y)
        
        lateral_cap = Polynomial.fit(velocity_y, lateral_g, deg=4)
        lateral_cap = lateral_cap.linspace()

        res = {
            "x": velocity_y,
            "y": lateral_g
        }

        dbfile = open('./lateral_x_y', 'ab')
        pickle.dump(res, dbfile)
        dbfile.close()

        fig, ax = plt.subplots()
        ax.plot(velocity_y, lateral_g, "o")
        ax.plot(lateral_cap[0], lateral_cap[1])
        plt.show()
        

    def vogel(self, x):
        deltar = 0
        
        delta = x[0]
        beta = x[1]
        AYP = x[2]

        V = math.sqrt(self.R * 9.81 * AYP)
        A_y = V**2 / self.R / 9.81  # (g's)
        WT = (
            A_y
            * self.DYN.cg_height
            * self.DYN.total_weight
            / statistics.mean([self.DYN.trackwidth_f, self.DYN.trackwidth_r])
        )

        WTF = WT * self.DYN.LLTD
        WTR = WT * (1 - self.DYN.LLTD)

        phif = A_y * self.DYN.roll_grad_f
        phir = A_y * self.DYN.roll_grad_f

        wfin = self.wf - WTF
        wfout = self.wf + WTF
        wrin = self.wr - WTR
        wrout = self.wr + WTR

        IA_f_in = (
            -self.DYN.trackwidth_f * math.sin(phif) / 2 * self.DYN.camber_gain_f
            - self.IA_0f
            - self.DYN.KPI_f * (1 - math.cos(delta))
            - self.DYN.caster_f * math.sin(delta)
            + phif
        )
        IA_f_out = (
            -self.DYN.trackwidth_f * math.sin(phif) / 2 * self.DYN.camber_gain_f
            + self.IA_0f
            + self.DYN.KPI_f * (1 - math.cos(delta))
            - self.DYN.caster_f * math.sin(delta)
            + phif
        )
        IA_r_in = (
            -self.DYN.trackwidth_r * math.sin(phir) / 2 * self.DYN.camber_gain_r
            - self.IA_0r
            - self.DYN.KPI_r * (1 - math.cos(deltar))
            - self.DYN.caster_f * math.sin(deltar)
            + phir
        )
        IA_r_out = (
            -self.DYN.trackwidth_r * math.sin(phir) / 2 * self.DYN.camber_gain_r
            + self.IA_0r
            + self.DYN.KPI_r * (1 - math.cos(deltar))
            - self.DYN.caster_f * math.sin(deltar)
            + phir
        )

        omega = V / self.R

        a_f = beta + self.a * omega / V - delta
        a_r = beta - self.b * omega / V

        F_fin = (
            self._MF52.Fy(a_f, wfin, -IA_f_in) * self.DYN.friction_scaling_y * math.cos(delta)
        )  # inputs = (rad Newtons rad)
        F_fout = (
            self._MF52.Fy(a_f, wfout, -IA_f_out)
            * self.DYN.friction_scaling_y
            * math.cos(delta)
        )

        F_xDrag = self.AERO.Cd * V**2 + (F_fin + F_fout) * math.sin(delta) / math.cos(
            delta
        )

        grip_val = np.interp(V, self.velocity_fit_x, self.grip_cap_fit_y)

        rscale = 1 - (F_xDrag / self.DYN.total_weight / (grip_val)) ** 2

        F_rin = self._MF52.Fy(a_r, wrin, -IA_r_in) * self.DYN.friction_scaling_y * rscale
        F_rout = self._MF52.Fy(a_r, wrout, -IA_r_out) * self.DYN.friction_scaling_y * rscale

        F_y = F_fin + F_fout + F_rin + F_rout

        M_z_diff = F_xDrag * self.PTN.diff_locked * self.DYN.trackwidth_r / 2
        M_z = (F_fin + F_fout) * self.a - (F_rin + F_rout) * self.b - M_z_diff

        AY = F_y / self.DYN.total_weight
        slipAngle = a_f - math.radians(-12)
        diff_AY = A_y - AY

        return [M_z, slipAngle, diff_AY]
