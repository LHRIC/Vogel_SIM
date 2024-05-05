import setups
import state_models

import math
import numpy as np

from utilities import MF52

class StateInput():
    def __init__(self, Ax=0, Ay=0, v=0, r=0, delta=0, beta=0):
        self.Ax = Ax
        self.Ay = Ay
        self.v = v

        self.r = r

        self.delta = delta
        self.beta = beta


class VehicleState():
    # TODO: Make a generic super class for setups
    def __init__(self, params: setups.Goose):
        self.params = params

        # Vehicle velocity
        self.v = 0

        # Radius of the turn that the vehicle is taking
        self.r = 0

        self.Ax = 0
        self.Ay = 0

        # Ackermann steer angle
        self.delta = 0

        # Vehicle sideslip angle
        self.beta = 0

        # Vehicle Yaw, Pitch, Roll angles - J670
        self.theta = 0
        self.phi_f = 0
        self.phi_r = 0

        # Sign Convention:
        # (+) - spring compression
        # (-) - spring extension
        self.fl_sus_dz = 0
        self.fr_sus_dz = 0
        self.rl_sus_dz = 0
        self.rr_sus_dz = 0

        self.fl_tire = state_models.TireState(params=params)
        self.fr_tire = state_models.TireState(params=params)
        self.rl_tire = state_models.TireState(params=params)
        self.rr_tire = state_models.TireState(params=params)

    def eval(self, state_in: StateInput):
        self.v = state_in.v
        self.r = state_in.r
        self.Ax = state_in.Ax
        self.Ay = state_in.Ay

        self.delta = state_in.delta
        self.beta = state_in.beta

        downforce = self.params.Cl * self.v**2
        # Define wheelbase aliases based on CG location
        a = self.params.wheelbase * (1 - self.params.weight_dist_f)
        b = self.params.wheelbase * self.params.weight_dist_f

        # Pitch and roll angles in radians
        self.theta = self.Ax * -1 * self.params.pitch_grad
        self.phi_f = self.Ay * self.params.roll_grad_f
        self.phi_r = self.Ay * self.params.roll_grad_r

        # Calculate rear sus drop (meters) due to pitching effects
        self.fl_sus_dz = -math.sin(abs(self.theta)) * self.params.wheelbase * (1 - self.params.weight_dist_f)
        self.rl_sus_dz = math.sin(abs(self.theta)) * self.params.wheelbase * self.params.weight_dist_f
        
        # Add on the effects of downforce
        self.fl_sus_dz += downforce * (1 - self.params.CoP) / 2 / self.params.ride_rate_f
        self.rl_sus_dz += downforce * (self.params.CoP) / 2 / self.params.ride_rate_r
        
        self.fr_sus_dz = self.fl_sus_dz
        self.rr_sus_dz = self.rl_sus_dz

        # Determine final suspension travel based on roll angle
        # NOTE: We will assume a left-handed turn for simplicity
        
        self.fl_sus_dz -= math.sin(self.phi_f) * self.params.trackwidth_f / 2
        self.rl_sus_dz -= math.sin(self.phi_f) * self.params.trackwidth_f / 2

        self.fr_sus_dz += math.sin(self.phi_f) * self.params.trackwidth_f / 2
        self.rr_sus_dz += math.sin(self.phi_f) * self.params.trackwidth_f / 2

        #RR - y = (0.000012) + (-0.065525x) + (-0.000119x**2) + ( 0.000000x**3)
        self.fl_sus_dz = self.fl_sus_dz * 1000
        self.rl_sus_dz = self.rl_sus_dz * 1000

        self.fr_sus_dz = self.fr_sus_dz * 1000
        self.rr_sus_dz = self.rr_sus_dz * 1000

        camber_gain = self.params.camber_gain_p

        bump_induced_camber_gain_f = -0.061177 * camber_gain
        bump_induced_camber_gain_r = -0.065569 * camber_gain

        camber_fl = bump_induced_camber_gain_f * self.fl_sus_dz
        camber_fr = bump_induced_camber_gain_f * self.fr_sus_dz
        camber_rl = bump_induced_camber_gain_r * self.rl_sus_dz
        camber_rr = bump_induced_camber_gain_r * self.rr_sus_dz

        IA_fl = (-0.032006) + bump_induced_camber_gain_f * self.fl_sus_dz
        IA_rl = (-0.029791) + bump_induced_camber_gain_r * self.rl_sus_dz
        IA_fr = (-0.032006) + bump_induced_camber_gain_f * self.fr_sus_dz
        IA_rr = (-0.029791) + bump_induced_camber_gain_r * self.rr_sus_dz

        #IA_fl = (0.000012) + (-0.061096 * self.fl_sus_dz) + (-0.000128 * self.fl_sus_dz**2) + (0.000000 * self.fl_sus_dz**3)
        #IA_rl = (0.000012) + (-0.065525 * self.rl_sus_dz) + (-0.000119 * self.rl_sus_dz**2) + ( 0.000000 * self.rl_sus_dz**3)

        #IA_fr = (0.000012) + (-0.061096 *  self.fr_sus_dz) + (-0.000128 *  self.fr_sus_dz**2) + (0.000000 * self.fr_sus_dz**3)
        #IA_rr = (0.000012) + (-0.065525 *  self.rr_sus_dz) + (-0.000119 *  self.rr_sus_dz**2) + ( 0.000000 * self.rr_sus_dz**3)
                
        IA_fl = math.radians(IA_fl)
        IA_rl = math.radians(IA_rl)

        IA_fr = math.radians(IA_fr)
        IA_rr = math.radians(IA_rr)

        w_rtire = (self.params.total_weight_r + downforce * (1 - self.params.CoP)) / 2
        w_rtire += (
                self.Ax * self.params.cg_height * self.params.total_weight / self.params.wheelbase / 2
            )
        
        w_ftire = (self.params.total_weight_f + downforce * (self.params.CoP)) / 2
        w_ftire -= (
                self.Ax * self.params.cg_height * self.params.total_weight / self.params.wheelbase / 2
            )
        
        lat_wt = (
            self.Ay
            * self.params.cg_height
            * self.params.total_weight
            / ((self.params.trackwidth_f + self.params.trackwidth_r) / 2)
        )

        lat_wt_f = lat_wt * self.params.LLTD
        lat_wt_r = lat_wt * (1 - self.params.LLTD)
        
        # NOTE: Left handed turn, so left tires are inner, right tires outer
        # weight transfer from inner tires to outer
        w_fl = w_ftire - lat_wt_f
        w_rl = w_rtire - lat_wt_r

        w_fr = w_ftire + lat_wt_f
        w_rr = w_rtire + lat_wt_r
        
        # Lazy zero-protection go crazy
        if(self.r == 0):
            self.alpha_f = 0
            self.alpha_r = 0
        else:
            self.alpha_f = self.beta + a * 1/self.r - self.delta
            self.alpha_r = self.beta - b * 1/self.r

        self.fl_tire.Fz = w_fl
        self.rl_tire.Fz = w_rl
        self.fr_tire.Fz = w_fr
        self.rr_tire.Fz = w_rr

        self.fl_tire.epsilon = IA_fl
        self.rl_tire.epsilon = IA_rl
        self.fr_tire.epsilon = IA_fr
        self.rr_tire.epsilon = IA_rr

        self.fl_tire.alpha = self.alpha_f
        self.rl_tire.alpha = self.alpha_r
        self.fr_tire.alpha = self.alpha_f
        self.rr_tire.alpha = self.alpha_r
        
        self.fl_tire.eval_Fx()
        self.rl_tire.eval_Fx()
        self.fr_tire.eval_Fx()
        self.rr_tire.eval_Fx()
        
        self.fl_tire.eval_Fy()
        self.rl_tire.eval_Fy()
        self.fr_tire.eval_Fy()
        self.rr_tire.eval_Fy()

