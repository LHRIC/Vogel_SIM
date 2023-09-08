import numpy as np
import math
from scipy.optimize import least_squares
from statistics import mean
import models
from fitting import csaps, polyfit
from utilities import MF52

class GGV:
    def __init__(self, AERO: models.AERO, DYN: models.DYN, PTN: models.PTN, gear_tot, v_max):
        self.AERO = AERO
        self.DYN = DYN
        self.PTN = PTN
        self._MF52 = MF52()

        self.gear_tot = gear_tot
        self.v_max = v_max

        self.velocity_range = np.arange(4, math.floor(self.v_max) + 1, 1)
        self.radii_range = np.arange(3.5, 36, 1.5)

        self.curr_gear = 1
        self.shift_count = 1
        self.expected_gears = []

        self._vogel_selector = 1
        self._calc_lateral = True

        self._grip_lim_accel = None
        self._power_lim_accel = None

        # These three FitFunctions describe the entire performance envelope (GGV) of the car.
        self.accel_capability = None
        self.cornering_capability = None
        self.braking_capability = None

        self.lateral_capability = np.array([
            1.38195832278988,
            1.42938584362690,
            1.46626343925345,
            1.49126646868585,
            1.51244972228054,
            1.53029970410364,
            1.54641562015666,
            1.56156587853348,
            1.57629799755505,
            1.59036217462828,
            1.60381424225666,
            1.61660679209661,
            1.62875493176357,
            1.64020405391575,
            1.65090601787886,
            1.66090586967413,
            1.66084693559365,
            1.65788498156999,
            1.65256859579085,
            1.64521365471666,
            1.63619883252664,
            1.62583858287551,
        ])


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
            # Calc load on each tire from static load and downforce
            wr = (self.DYN.total_weight_r + downforce * (1 - self.AERO.CoP)) / 2
            
            # Add on the effects of longitudinal weight transfer
            wr += (
                Ax * self.DYN.cg_height * self.DYN.total_weight / self.DYN.wheelbase / 2
            )

            #Calculate new camber from camber gain
            IA_r = (
                self.DYN.wheelbase * math.sin(pitch) / 2 * self.DYN.camber_gain_r
                + IA_0r
            )

            fxr = []
            for sl in np.arange(0, 1.01, 0.01):
                Fx = self._MF52.Fx(-wr, -IA_r, sl) * self.DYN.friction_scaling_x
                fxr.append(Fx)
            
            # Get the maximum possible force experienced by the tire 
            FXR = max(fxr)
            FX = abs(2 * FXR)

            # Calculate potential acceleration if the force was generated at both rear contact patches
            AX = FX / self.DYN.total_weight  # THIS IS IN G'S, SEE ABOVE

            #Iterate until acceleration converges
            A_x_diff = AX - Ax

        return AX

    def calc_power_lim_max_accel(self, v):
        
        gear_idx = 0
        rpm = self.PTN.shiftpoint
        rpm_diff = 1000

        # Short little accel simulation, determine what gear the car will be in
        # if it had to accelerate to this velocity

        #TODO: Revisit this calculation, not entirely sure that it is accurate
        while rpm >= self.PTN.shiftpoint and rpm_diff > 1e-6:
            total_red = (
                self.PTN.gear_ratios[gear_idx]
                * self.PTN.final_drive
                * self.PTN.primary_reduction
            )
            gear_idx += 1
            gear_idx = min(self.PTN.num_gears - 1, gear_idx)

            rpm_diff = abs(rpm -  v * total_red / self.DYN.tire_radius * 60 / (2 * math.pi))

            rpm = v * total_red / self.DYN.tire_radius * 60 / (2 * math.pi)

        # Determine torque at the crankshaft
        crankshaft_torque = np.interp(rpm, self.PTN.rpm_range, self.PTN.torque_curve)
        wheel_torque = crankshaft_torque * total_red * self.PTN.drivetrain_losses
        Fx = wheel_torque / self.DYN.tire_radius

        # gear_idx is what gear we are in after acceling to the input velocity
        return (Fx, gear_idx)

    def calc_lateral_accel(self, R):
        # Almost like an estimation of the over/understeer balance of the car
        AYP = 0.5

        self.a = self.DYN.wheelbase * (1 - self.DYN.weight_dist_f)
        self.b = self.DYN.wheelbase * self.DYN.weight_dist_f
        self.R = R
        
        # Initial guess for velocity from radii and lateral acceleration guess 
        V = math.sqrt(R * 9.81 * AYP)

        # Down (Lift) Force calculated from velo guess
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

        # Guess initial ackermann steer angle
        delta = self.DYN.wheelbase / R

        # Assume sideslip starts at 0
        beta = 0

        # a, b, R, wf, wr, IA_0f, IA_0r, 0, velocity_fit_x, grip_cap_fit_y
        x0 = [delta, beta, AYP]
        lb = [0.01, -0.3, 0.1]
        ub = [0.5, 0.3, 3]

        # print(self.vogel(lb), end=", ")
        # print(self.vogel(ub))

        self._vogel_selector = 1
        x = least_squares(self.vogel, x0, bounds=(lb, ub), method="trf")

        delta = x.x[0]
        beta = x.x[1]
        AYP = x.x[2]

        self._vogel_selector = 0
        eval_vogel = self.vogel([delta, beta, AYP])

        # delta_l.append(delta)
        # beta_l.append(beta)
        # AYP_l.append(AYP)

        return eval_vogel[0]
    
    def calc_decel(self, v):
        downforce = self.AERO.Cl * v**2  # Downforce: Newtons
        # dxf, dxr
        sus_drop_f = downforce * (self.AERO.CoP) / 2 / self.DYN.ride_rate_f
        sus_drop_r = downforce * (1 - self.AERO.CoP) / 2 / self.DYN.ride_rate_r

        # IA_0r, IA_0f
        IA_0f = self.DYN.static_camber_f - sus_drop_f * self.DYN.camber_gain_f
        IA_0r = self.DYN.static_camber_r - sus_drop_r * self.DYN.camber_gain_r

        Ax = 0.99
        while A_x_diff > 0:
            Ax += 0.01
            pitch = Ax * self.DYN.pitch_grad

            wf = (self.DYN.total_weight_f + downforce * (self.AERO.CoP)) / 2
            wr = (self.DYN.total_weight_r + downforce * (1 - self.AERO.CoP)) / 2
            wf += (
                Ax * self.DYN.cg_height * self.DYN.total_weight / self.DYN.wheelbase / 2
            )
            wr -= (
                Ax * self.DYN.cg_height * self.DYN.total_weight / self.DYN.wheelbase / 2
            )

            IA_f = (-self.DYN.wheelbase * 12 * math.sin(pitch) / 2 * self.DYN.camber_gain_f) + IA_0f
            IA_r = (self.DYN.wheelbase * 12 * math.sin(pitch) / 2 * self.DYN.camber_gain_r) + IA_0r

            fx_f = []
            fx_r = []

            for sl in np.arange(-0.23, 0.01, 0.01):
                Fx_f = self._MF52.Fx(-wf, -IA_f, sl) * self.DYN.friction_scaling_x
                Fx_r = self._MF52.Fx(-wr, -IA_r, sl) * self.DYN.friction_scaling_x
                fx_f.append(Fx_f)
                fx_r.append(Fx_r)

            FXF = min(fx_f)
            FXR = min(fx_r)

            FX = abs(2 * FXF + 2 * FXR)
            AX = FX / self.DYN.total_weight  # THIS IS IN G'S, SEE ABOVE
            A_x_diff = AX - Ax
        return Ax

    
    def generate(self):
        power_lim_a = np.empty((len(self.velocity_range),))
        grip_lim_a = np.empty((len(self.velocity_range),))
        accel_cap = np.empty((len(self.velocity_range),))

        '''The gear that you start each velocity iteration in'''
        for idx, v in enumerate(self.velocity_range):
            print(f"Calculating: Long. accel capability for {v} m/s")
            Ax_r = self.calc_grip_lim_max_accel(v)
            grip_lim_a[idx] = (Ax_r)

            FX_r, gear_idx = self.calc_power_lim_max_accel(max(7.5, v))
            self.expected_gears.append(gear_idx)

            # TODO: Investigate whether or not the grip limited case should subtract drag as well.
            FX_r -= self.AERO.Cd * v**2  #Take drag into account
            AX_r = FX_r / self.DYN.total_weight
            power_lim_a[idx] = (AX_r)

            accel_cap[idx] = (min(Ax_r, AX_r))
        

        self._grip_lim_accel = polyfit(self.velocity_range, grip_lim_a, 3)
        self._power_lim_accel = csaps(self.velocity_range, power_lim_a)
        self.accel_capability = csaps(self.velocity_range, accel_cap)

        lateral_g = np.empty((len(self.radii_range),))
        if(self._calc_lateral):
            for idx, R in enumerate(self.radii_range):
                print(f"Calculating: Lat. accel capability for {R}/{self.radii_range[-1]} m")
                lateral_g[idx] = (self.calc_lateral_accel(R))

            lateral_g = np.array(lateral_g)
            accel_y = lateral_g * 9.81
            velocity_y = np.multiply(accel_y, self.radii_range)
            velocity_y = np.sqrt(velocity_y)

            self.lateral_capability = polyfit(velocity_y, lateral_g, degree=4)

            #self.lateral_capability.plot()

        else:
            print("WARNING: Loading precalculated lateral envelope")
            lateral_g = self.lateral_capability
            velocity_y = lateral_g * 9.81
            velocity_y = np.multiply(velocity_y, self.radii_range)
            velocity_y = np.sqrt(velocity_y)

            self.lateral_capability = polyfit(velocity_y, lateral_g, degree=4)
            
        # Defines the max cornering velocity vs radii *based on lateral acceleration capacity*
        # Not based on steady-state cornering acceleration
        self.cornering_capability = polyfit(self.radii_range, velocity_y, 4)
        
        braking_g = np.empty((len(self.velocity_range),))
        for idx, v in enumerate(self.velocity_range):
            print(f"Calculating: Long. braking capability for {v} m/s")
            Ax = self.calc_grip_lim_max_accel(v)
            braking_g[idx] = (Ax)
        self.braking_capability = polyfit(self.velocity_range, braking_g, degree=4)



    '''Calculates the maximum lateral acceleration for a neural steer car, given a predetermined turn radius'''
    def vogel(self, x):
        deltar = 0

        delta = x[0]
        beta = x[1]
        AYP = x[2]

        V = math.sqrt(self.R * 9.81 * AYP)
        A_y = AYP

        '''Calculate the Lateral Load Transfer from cornering in Newtons'''
        WT = (
            A_y
            * self.DYN.cg_height
            * self.DYN.total_weight
            / mean([self.DYN.trackwidth_f, self.DYN.trackwidth_r])
        )

        '''Distribute the Load Transfer to the F/R based on LLTD'''
        WTF = WT * self.DYN.LLTD
        WTR = WT * (1 - self.DYN.LLTD)
        
        phif = A_y * self.DYN.roll_grad_f
        phir = A_y * self.DYN.roll_grad_f


        '''Apply weight transfer to each tire accordingly'''
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

        # Using guesses for sideslip and Ackermann steer angle
        # Try and calculate tire slip angles in the front and the rear
        a_f = beta + self.a * omega / V - delta
        a_r = beta - self.b * omega / V

        F_fin = (
            self._MF52.Fy(a_f, wfin, -IA_f_in)
            * self.DYN.friction_scaling_y
            * math.cos(delta)
        )  # inputs = (rad Newtons rad)
        F_fout = (
            self._MF52.Fy(a_f, wfout, -IA_f_out)
            * self.DYN.friction_scaling_y
            * math.cos(delta)
        )

        F_xDrag = self.AERO.Cd * V**2 + (F_fin + F_fout) * math.sin(delta) / math.cos(
            delta
        )

        grip_lim_accel = self._grip_lim_accel.evaluate(V)
        #np.interp(V, self.velocity_fit_x, self.grip_cap_fit_y)


        '''Calculate the grip cost of overcoming vehicle drag'''
        rscale = 1 - (F_xDrag / self.DYN.total_weight / (grip_lim_accel)) ** 2

        F_rin = (
            self._MF52.Fy(a_r, wrin, -IA_r_in) * self.DYN.friction_scaling_y * rscale
        )
        F_rout = (
            self._MF52.Fy(a_r, wrout, -IA_r_out) * self.DYN.friction_scaling_y * rscale
        )

        F_y = F_fin + F_fout + F_rin + F_rout
        f_xplt = (F_fin + F_fout) * math.sin(delta) / math.cos(delta) / 400

        # This is the moment generated by the diff
        M_z_diff = F_xDrag * self.PTN.diff_locked * self.DYN.trackwidth_r / 2
        
        # This is the moment acting on the CG (causing under/oversteer)
        M_z = (F_fin + F_fout) * self.a - (F_rin + F_rout) * self.b - M_z_diff

        AY = F_y / self.DYN.total_weight

        # Tries to minimize the objective function such that slip angle is kept at 12 degrees (bounds of tire model)
        slipAngle = a_f - math.radians(self.DYN.ideal_slip)
        diff_AY = A_y - AY

        if self._vogel_selector == 1:
            return [M_z, slipAngle, diff_AY]
        else:
            return [A_y, f_xplt, a_f, a_r]