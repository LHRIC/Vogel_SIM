import models
import state_models
import math
import numpy as np

from utilities import MF52

class VehicleState():
    def __init__(self, AERO: models.AERO, DYN: models.DYN, PTN: models.PTN, Ax, Ay):
        self.AERO = AERO
        self.DYN = DYN
        self.PTN = PTN

        # Vehicle velocity
        self.v = 35

        # Radius of the turn that the vehicle is taking
        self.r = 15

        self.Ax = Ax
        self.Ay = Ay

        # Vehicle Yaw, Pitch, Roll angles - J670
        self.theta = 0
        self.phi_f = 0
        self.phi_r = 0

        # Ackermann steer angle
        self.delta = 0.2

        # Vehicle sideslip angle
        self.beta = 0.1

        # Sign Convention:
        # (+) - spring compression
        # (-) - spring extension
        self.fl_sus_dz = 0
        self.fr_sus_dz = 0
        self.rl_sus_dz = 0
        self.rr_sus_dz = 0

        self.fl_tire = state_models.TireState()
        self.fr_tire = state_models.TireState()
        self.rl_tire = state_models.TireState()
        self.rr_tire = state_models.TireState()

    def eval(self):
        downforce = self.AERO.Cl * self.v**2
        # Define wheelbase aliases based on CG location
        a = self.DYN.wheelbase * (1 - self.DYN.weight_dist_f)
        b = self.DYN.wheelbase * self.DYN.weight_dist_f

        # Pitch and roll angles in radians
        self.theta = self.Ax * -1 * self.DYN.pitch_grad
        self.phi_f = self.Ay * self.DYN.roll_grad_f
        self.phi_r = self.Ay * self.DYN.roll_grad_r

        # Calculate rear sus drop (meters) due to pitching effects
        self.fl_sus_dz = -math.sin(abs(self.theta)) * self.DYN.wheelbase * (1 - self.DYN.weight_dist_f)
        self.rl_sus_dz = math.sin(abs(self.theta)) * self.DYN.wheelbase * self.DYN.weight_dist_f
        
        # Add on the effects of downforce
        self.fl_sus_dz += downforce * (self.AERO.CoP) / 2 / self.DYN.ride_rate_f
        self.rl_sus_dz += downforce * (1 - self.AERO.CoP) / 2 / self.DYN.ride_rate_r
        
        self.fr_sus_dz = self.fl_sus_dz
        self.rr_sus_dz = self.rl_sus_dz

        # Determine final suspension travel based on roll angle
        # NOTE: We will assume a left-handed turn for simplicity
        
        self.fl_sus_dz -= math.sin(self.phi_f) * self.DYN.trackwidth_f / 2
        self.rl_sus_dz -= math.sin(self.phi_f) * self.DYN.trackwidth_f / 2

        self.fr_sus_dz += math.sin(self.phi_f) * self.DYN.trackwidth_f / 2
        self.rr_sus_dz += math.sin(self.phi_f) * self.DYN.trackwidth_f / 2

        #RR - y = (0.000012) + (-0.065525x) + (-0.000119x**2) + ( 0.000000x**3)
        self.fl_sus_dz = self.fl_sus_dz * 1000
        self.rl_sus_dz = self.rl_sus_dz * 1000

        self.fr_sus_dz = self.fr_sus_dz * 1000
        self.rr_sus_dz = self.rr_sus_dz * 1000

        IA_fl = (0.000012) + (-0.061096 * self.fl_sus_dz) + (-0.000128 * self.fl_sus_dz**2) + (0.000000 * self.fl_sus_dz**3)
        IA_rl = (0.000012) + (-0.065525 * self.rl_sus_dz) + (-0.000119 * self.rl_sus_dz**2) + ( 0.000000 * self.rl_sus_dz**3)

        IA_fr = (0.000012) + (-0.061096 *  self.fr_sus_dz) + (-0.000128 *  self.fr_sus_dz**2) + (0.000000 * self.fr_sus_dz**3)
        IA_rr = (0.000012) + (-0.065525 *  self.rr_sus_dz) + (-0.000119 *  self.rr_sus_dz**2) + ( 0.000000 * self.rr_sus_dz**3)
                
        IA_fl = math.radians(IA_fl)
        IA_rl = math.radians(IA_rl)

        IA_fr = math.radians(IA_fr)
        IA_rr = math.radians(IA_rr)

        w_rtire = (self.DYN.total_weight_r + downforce * (1 - self.AERO.CoP)) / 2
        w_rtire += (
                self.Ax * self.DYN.cg_height * self.DYN.total_weight / self.DYN.wheelbase / 2
            )
        
        w_ftire = (self.DYN.total_weight_f + downforce * (self.AERO.CoP)) / 2
        w_ftire -= (
                self.Ax * self.DYN.cg_height * self.DYN.total_weight / self.DYN.wheelbase / 2
            )
        
        lat_wt = (
            self.Ay
            * self.DYN.cg_height
            * self.DYN.total_weight
            / ((self.DYN.trackwidth_f + self.DYN.trackwidth_r) / 2)
        )

        lat_wt_f = lat_wt * self.DYN.LLTD
        lat_wt_r = lat_wt * (1 - self.DYN.LLTD)
        
        # NOTE: Left handed turn, so left tires are inner, right tires outer
        # weight transfer from inner tires to outer
        w_fl = w_ftire - lat_wt_f
        w_rl = w_rtire - lat_wt_r

        w_fr = w_ftire + lat_wt_f
        w_rr = w_rtire + lat_wt_r

        print(w_fl, w_rl, w_fr, w_rr)
        
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
        
        _MF52 = MF52()

        fx_fl = []
        for sl in np.arange(0, 1.01, 0.01):
            Fx = _MF52.Fx(w_fl, IA_fl, sl) * self.DYN.friction_scaling_x
            fx_fl.append(Fx)
        
        fx_rl = []
        for sl in np.arange(0, 1.01, 0.01):
            Fx = _MF52.Fx(w_rl, IA_rl, sl) * self.DYN.friction_scaling_x
            fx_rl.append(Fx)
        
        fx_fr = []
        for sl in np.arange(0, 1.01, 0.01):
            Fx = _MF52.Fx(w_fr, IA_fr, sl) * self.DYN.friction_scaling_x
            fx_fr.append(Fx)
        
        fx_rr = []
        for sl in np.arange(0, 1.01, 0.01):
            Fx = _MF52.Fx(w_rr, IA_rr, sl) * self.DYN.friction_scaling_x
            fx_rr.append(Fx)

        fx_fl = max(fx_fl)
        fx_rl = max(fx_rl)
        fx_fr = max(fx_fr)
        fx_rr = max(fx_rr)
        
        fy_fl = self.fl_tire.Fy() * self.DYN.friction_scaling_y
        fy_rl = self.rl_tire.Fy() * self.DYN.friction_scaling_y
        fy_fr = self.fr_tire.Fy() * self.DYN.friction_scaling_y
        fy_rr = self.rr_tire.Fy() * self.DYN.friction_scaling_y