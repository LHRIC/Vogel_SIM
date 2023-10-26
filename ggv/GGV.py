import numpy as np
import math
from scipy.optimize import least_squares
from statistics import mean
from fitting import csaps, polyfit
from utilities import MF52
import setups
import state_models

class GGV:
    def __init__(self, params: setups.VehicleSetup, gear_tot, v_max):
        self.params = params

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

        self.vehicle_state = state_models.VehicleState(params=self.params)

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

    def calc_grip_lin_max_accel(self, v):
        vehicle_state = self.vehicle_state
        A_x_diff = 1
        Ax = 0
        AX = 0
        while A_x_diff > 1e-6:
            Ax += 0.01
            state_in = state_models.StateInput(Ax=Ax, Ay=0, v=v, r=0, delta=0, beta=0)
            vehicle_state.eval(state_in=state_in)

            FX_rear = vehicle_state.rl_tire.Fx + vehicle_state.rr_tire.Fx
            AX = FX_rear / self.params.total_weight

            A_x_diff = AX - Ax
    
        return AX
    
    def calc_power_lim_max_accel(self, v):
        
        gear_idx = 0
        rpm = self.params.shiftpoint
        rpm_diff = 1000

        # Short little accel simulation, determine what gear the car will be in
        # if it had to accelerate to this velocity

        #TODO: Revisit this calculation, not entirely sure that it is accurate
        while rpm >= self.params.shiftpoint and rpm_diff > 1e-6:
            total_red = (
                self.params.gear_ratios[gear_idx]
                * self.params.final_drive
                * self.params.primary_reduction
            )
            gear_idx += 1
            gear_idx = min(self.params.num_gears - 1, gear_idx)

            rpm_diff = abs(rpm -  v * total_red / self.params.tire_radius * 60 / (2 * math.pi))

            rpm = v * total_red / self.params.tire_radius * 60 / (2 * math.pi)

        # Determine torque at the crankshaft
        crankshaft_torque = np.interp(rpm, self.params.rpm_range, self.params.torque_curve)
        wheel_torque = crankshaft_torque * total_red * self.params.drivetrain_losses
        Fx = wheel_torque / self.params.tire_radius

        # gear_idx is what gear we are in after acceling to the input velocity
        return (Fx, gear_idx)

    def calc_lateral_accel(self, R):
        AYP = 0.5

        self.a = self.params.wheelbase * (1 - self.params.weight_dist_f)
        self.b = self.params.wheelbase * self.params.weight_dist_f
        self.R = R
        
        # Initial guess for velocity from radii and lateral acceleration guess 
        V = math.sqrt(R * 9.81 * AYP)

        # Down (Lift) Force calculated from velo guess
        LF = self.params.Cl * V**2

        # Update suspension travel
        dxf = LF * self.params.CoP / 2 / self.params.ride_rate_f
        dxr = LF * (1 - self.params.CoP) / 2 / self.params.ride_rate_r

        # Get static camber from suspension heave
        self.IA_0f = self.params.static_camber_f - dxf * self.params.camber_gain_f
        self.IA_0r = self.params.static_camber_r - dxr * self.params.camber_gain_r

        # Get loads on each wheel
        self.wf = (self.params.total_weight_f + LF * self.params.CoP) / 2
        self.wr = (self.params.total_weight_r + LF * (1 - self.params.CoP)) / 2

        # Guess initial ackermann steer angle
        delta = self.params.wheelbase / R

        # Assume sideslip starts at 0
        beta = 0

        # a, b, R, wf, wr, IA_0f, IA_0r, 0, velocity_fit_x, grip_cap_fit_y
        x0 = [delta, beta, AYP]
        lb = [0.01, -0.3, 0.1]
        ub = [0.5, 0.3, 3]

        # print(self.vogel(lb), end=", ")
        # print(self.vogel(ub))

        self._vogel_selector = 1
        x = least_squares(self.vogel, x0, bounds=(lb, ub), method="trf", verbose=1)

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
        vehicle_state = self.vehicle_state
        A_x_diff = 1 # Initial condition to start the iteration
        Ax = 0
        AX = 0
        while A_x_diff > 0:
            Ax -= 0.01
            state_in = state_models.StateInput(Ax=Ax, Ay=0, v=v, r=0, delta=0, beta=0)
            vehicle_state.eval(state_in=state_in)

            FX_front = vehicle_state.fl_tire.Fx + vehicle_state.fr_tire.Fx
            FX_rear = vehicle_state.rl_tire.Fx + vehicle_state.rr_tire.Fx

            FX = -1 * (FX_front + FX_rear)
            AX = FX / self.params.total_weight

            A_x_diff = AX - Ax
        return AX

    def generate(self):
        
        power_lim_a = np.empty((len(self.velocity_range),))
        grip_lim_a = np.empty((len(self.velocity_range),))
        accel_cap = np.empty((len(self.velocity_range),))

        '''The gear that you start each velocity iteration in'''
        for idx, v in enumerate(self.velocity_range):
            print(f"Calculating: Long. accel capability for {v} m/s")
            #Ax_r = self.calc_grip_lim_max_accel(v)
            Ax_r = self.calc_grip_lin_max_accel(v)
            
            grip_lim_a[idx] = (Ax_r)

            FX_r, gear_idx = self.calc_power_lim_max_accel(max(7.5, v))
            self.expected_gears.append(gear_idx)

            # TODO: Investigate whether or not the grip limited case should subtract drag as well.
            FX_r -= self.params.Cd * v**2  #Take drag into account
            AX_r = FX_r / self.params.total_weight
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
            Ax = self.calc_decel(v)
            braking_g[idx] = (Ax)
        self.braking_capability = polyfit(self.velocity_range, braking_g, degree=4)

    
    '''Calculates the maximum lateral acceleration for a neural steer car, given a predetermined turn radius'''
    def vogel(self, x):
        vehicle_state = self.vehicle_state

        a = self.params.wheelbase * (1 - self.params.weight_dist_f)
        b = self.params.wheelbase * self.params.weight_dist_f

        delta = x[0]
        beta = x[1]
        AYP = x[2]
        R = self.R

        V = math.sqrt(R * 9.81 * AYP)
        A_y = AYP

        state_in = state_models.StateInput(Ax=0, Ay=A_y, v=V, r=R, delta=delta, beta=beta)
        vehicle_state.eval(state_in=state_in)

        F_f_in = vehicle_state.fl_tire.Fy * math.cos(delta)
        F_f_out = vehicle_state.fr_tire.Fy * math.cos(delta)
        F_xDrag = self.params.Cd * V**2 + (F_f_in + F_f_out) * math.sin(delta) / math.cos(
            delta
        )

        grip_lim_accel = self._grip_lim_accel.evaluate(V)

        rscale = 1 - (F_xDrag / self.params.total_weight / (grip_lim_accel)) ** 2

        F_r_in = vehicle_state.rl_tire.Fy * rscale
        F_r_out = vehicle_state.rr_tire.Fy * rscale

        F_y = F_f_in + F_f_out + F_r_in + F_r_out
        f_xplt = (F_f_in + F_f_out) * math.sin(delta) / math.cos(delta) / 400

        # This is the moment generated by the diff
        M_z_diff = F_xDrag * self.params.diff_locked * self.params.trackwidth_r / 2
        
        # This is the moment acting on the CG (causing under/oversteer)
        M_z = (F_f_in + F_f_out) * a - (F_r_in + F_r_out) * b - M_z_diff

        AY = F_y / self.params.total_weight

        # Tries to minimize the objective function such that slip angle is kept at 12 degrees (bounds of tire model)
        slipAngle = vehicle_state.alpha_f - math.radians(-12)
        diff_AY = A_y - AY

        if self._vogel_selector == 1:
            return [M_z, slipAngle, diff_AY]
        else:
            return [A_y, f_xplt, vehicle_state.alpha_f, vehicle_state.alpha_r]