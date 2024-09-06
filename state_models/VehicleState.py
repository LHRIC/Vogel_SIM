import setups
import state_models

import math
import numpy as np

from utilities import MF52

class StateInput():
    #max ay and max az and then itirate that through friction elipse
    
    def __init__(self, Ax=0, Ay=0, v=0, r=0, delta=0, beta=0):
        self.Ax = Ax
        self.Ay = Ay
        self.v = v

        self.r = r

        self.delta = delta
        self.beta = beta


class VehicleState():
    # TODO: Make a generic super class for setups
    def __init__(self, params: setups.Panda):
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
 
        ## Weight transfer from roll
        # Axle equivalent mass
        m_f = self.params.total_weight_f #N
        m_r = self.params.total_weight_r #N
        m_t = self.params.total_weight #N
        # Intermediate calcs
        k_phi_ratio = self.params.k_phi_f/(self.params.k_phi_f+self.params.k_phi_r)
        roll_lever_f = self.params.cg_height-self.params.rollc_f
        roll_lever_r = self.params.cg_height-self.params.rollc_r
        # Geometric and Elastic weight transfer
        dFz_geom_roll_f = m_f*self.params.rollc_f*self.Ay/self.params.trackwidth_f
        dFz_geom_roll_r = m_r*self.params.rollc_r*self.Ay/self.params.trackwidth_r
        dFz_elas_roll_f = k_phi_ratio*(m_f*roll_lever_f+m_r*roll_lever_r)*self.Ay/self.params.trackwidth_f
        dFz_elas_roll_r = (1-k_phi_ratio)*(m_f*roll_lever_f+m_r*roll_lever_r)*self.Ay/self.params.trackwidth_r

        # Weight transfer from pitch
        dFz_tot_pitch = self.params.cg_height*m_t*self.Ax/self.params.wheelbase
        if dFz_tot_pitch >= 0:
            dFz_elas_pitch_r = dFz_tot_pitch*(1-self.params.asquat)
            dFz_elas_pitch_f = -dFz_tot_pitch
        else:
            dFz_elas_pitch_r = dFz_tot_pitch
            dFz_elas_pitch_f = -dFz_tot_pitch*(1-self.params.adive)
        dFz_geom_pitch_r = dFz_tot_pitch - dFz_elas_pitch_r
        dFz_geom_pitch_f = -dFz_tot_pitch + dFz_elas_pitch_f

        # Downforce (assumed always elastic)
        dFz_aero_f = self.params.CoP*self.params.Cl*self.v**2
        dFz_aero_r = (1-self.params.CoP)*self.params.Cl*self.v**2

        # Net elastic loads (contributing to spring compression)
        Fz_elas_FL = -dFz_elas_roll_f + dFz_elas_pitch_f/2 + dFz_aero_f/2
        Fz_elas_FR = dFz_elas_roll_f + dFz_elas_pitch_f/2 + dFz_aero_f/2
        Fz_elas_RL = -dFz_elas_roll_r + dFz_elas_pitch_r/2 + dFz_aero_r/2
        Fz_elas_RR = dFz_elas_roll_r + dFz_elas_pitch_r/2 + dFz_aero_r/2
        
        # Net loads
        Fz_tot_FL = self.params.total_weight_f/2 + Fz_elas_FL - dFz_geom_roll_f + dFz_geom_pitch_f/2
        Fz_tot_FR = self.params.total_weight_f/2 + Fz_elas_FR + dFz_geom_roll_f + dFz_geom_pitch_f/2
        Fz_tot_RL = self.params.total_weight_r/2 + Fz_elas_RL - dFz_geom_roll_r + dFz_geom_pitch_r/2
        Fz_tot_RR = self.params.total_weight_r/2 + Fz_elas_RR + dFz_geom_roll_r + dFz_geom_pitch_r/2

        # Suspension travel (mm)
        dz_FL = Fz_elas_FL/(self.params.ride_rate_f/1000)
        dz_FR = Fz_elas_FR/(self.params.ride_rate_f/1000)
        dz_RL = Fz_elas_RL/(self.params.ride_rate_r/1000)
        dz_RR = Fz_elas_RR/(self.params.ride_rate_r/1000)

        # Camber + gain (deg)
        camber_FL = self.params.static_camber_f + dz_FL*self.params.camber_gain_f
        camber_FR = self.params.static_camber_f + dz_FR*self.params.camber_gain_f
        camber_RL = self.params.static_camber_r + dz_RL*self.params.camber_gain_r
        camber_RR = self.params.static_camber_r + dz_RR*self.params.camber_gain_r

        # Body roll (degrees)
        phi_f = math.degrees(math.asin((dz_FR-dz_FL)/(self.params.trackwidth_f*1000)))
        phi_r = math.degrees(math.asin((dz_RR-dz_RL)/(self.params.trackwidth_f*1000)))
        self.phi = (phi_f+phi_r)/2 # They should be the same surely
        print('body roll:')
        print(phi_f, phi_r)
        # Inclination angle
        IA_FL = camber_FL - self.phi
        IA_FR = camber_FR + self.phi
        IA_RL = camber_RL - self.phi
        IA_RR = camber_RR + self.phi

        ## Pass to tire model
        # Normal loads
        self.fl_tire.Fz = Fz_tot_FL
        self.fr_tire.Fz = Fz_tot_FR
        self.rl_tire.Fz = Fz_tot_RL
        self.rr_tire.Fz = Fz_tot_RR
        # Inclination angle
        self.fl_tire.epsilon = math.radians(IA_FL)
        self.fr_tire.epsilon = math.radians(IA_FR)
        self.rl_tire.epsilon = math.radians(IA_RL)
        self.rr_tire.epsilon = math.radians(IA_RR)
        print('Camber:')
        print(camber_FL, camber_FR, camber_RL, camber_RR)
        print('DZ:')
        print(dz_FL,dz_FR,dz_RL,dz_RR)
        print('FZ elastic:')
        print(Fz_elas_FL, Fz_elas_FR, Fz_elas_RL, Fz_elas_RR)
        print('IA:')
        print(IA_FL,IA_FR,IA_RL,IA_RR)
        # downforce = self.params.Cl * self.v**2

        # Define wheelbase aliases based on CG location
        a = self.params.wheelbase * (1 - self.params.weight_dist_f)
        b = self.params.wheelbase * self.params.weight_dist_f

        # # Pitch and roll angles in radians
        # self.theta = self.Ax * -1 * self.params.pitch_grad
        # self.phi_f = self.Ay * self.params.roll_grad_f
        # self.phi_r = self.Ay * self.params.roll_grad_r

        # # Calculate rear sus drop (meters) due to pitching effects
        # self.fl_sus_dz = -math.sin(abs(self.theta)) * self.params.wheelbase * (1 - self.params.weight_dist_f)
        # self.rl_sus_dz = math.sin(abs(self.theta)) * self.params.wheelbase * self.params.weight_dist_f
        
        # # Add on the effects of downforce
        # self.fl_sus_dz += downforce * (1 - self.params.CoP) / 2 / self.params.ride_rate_f
        # self.rl_sus_dz += downforce * (self.params.CoP) / 2 / self.params.ride_rate_r
        
        # self.fr_sus_dz = self.fl_sus_dz
        # self.rr_sus_dz = self.rl_sus_dz

        # # Determine final suspension travel based on roll angle
        # # NOTE: We will assume a left-handed turn for simplicity
        
        # self.fl_sus_dz -= math.sin(self.phi_f) * self.params.trackwidth_f / 2
        # self.rl_sus_dz -= math.sin(self.phi_f) * self.params.trackwidth_f / 2

        # self.fr_sus_dz += math.sin(self.phi_f) * self.params.trackwidth_f / 2
        # self.rr_sus_dz += math.sin(self.phi_f) * self.params.trackwidth_f / 2

        # #RR - y = (0.000012) + (-0.065525x) + (-0.000119x**2) + ( 0.000000x**3)
        # self.fl_sus_dz = self.fl_sus_dz * 1000
        # self.rl_sus_dz = self.rl_sus_dz * 1000

        # self.fr_sus_dz = self.fr_sus_dz * 1000
        # self.rr_sus_dz = self.rr_sus_dz * 1000

        # camber_gain = self.params.camber_gain_p

        # bump_induced_camber_gain_f = -0.061177 * camber_gain
        # bump_induced_camber_gain_r = -0.065569 * camber_gain

        # camber_fl = bump_induced_camber_gain_f * self.fl_sus_dz
        # camber_fr = bump_induced_camber_gain_f * self.fr_sus_dz
        # camber_rl = bump_induced_camber_gain_r * self.rl_sus_dz
        # camber_rr = bump_induced_camber_gain_r * self.rr_sus_dz

        # IA_fl = (-0.032006) + bump_induced_camber_gain_f * self.fl_sus_dz + (-1)*self.phi_f
        # IA_rl = (-0.029791) + bump_induced_camber_gain_r * self.rl_sus_dz + (-1)*self.phi_r
        # IA_fr = (-0.032006) + bump_induced_camber_gain_f * self.fr_sus_dz + self.phi_f
        # IA_rr = (-0.029791) + bump_induced_camber_gain_r * self.rr_sus_dz + self.phi_r

        # #IA_fl = (0.000012) + (-0.061096 * self.fl_sus_dz) + (-0.000128 * self.fl_sus_dz**2) + (0.000000 * self.fl_sus_dz**3)
        # #IA_rl = (0.000012) + (-0.065525 * self.rl_sus_dz) + (-0.000119 * self.rl_sus_dz**2) + ( 0.000000 * self.rl_sus_dz**3)

        # #IA_fr = (0.000012) + (-0.061096 *  self.fr_sus_dz) + (-0.000128 *  self.fr_sus_dz**2) + (0.000000 * self.fr_sus_dz**3)
        # #IA_rr = (0.000012) + (-0.065525 *  self.rr_sus_dz) + (-0.000119 *  self.rr_sus_dz**2) + ( 0.000000 * self.rr_sus_dz**3)
                
        # IA_fl = math.radians(IA_fl)
        # IA_rl = math.radians(IA_rl)

        # IA_fr = math.radians(IA_fr)
        # IA_rr = math.radians(IA_rr)

        # w_rtire = (self.params.total_weight_r + downforce * (1 - self.params.CoP)) / 2
        # w_rtire += (
        #         self.Ax * self.params.cg_height * self.params.total_weight / self.params.wheelbase / 2
        #     )
        
        # w_ftire = (self.params.total_weight_f + downforce * (self.params.CoP)) / 2
        # w_ftire -= (
        #         self.Ax * self.params.cg_height * self.params.total_weight / self.params.wheelbase / 2
        #     )
        
        # lat_wt = (
        #     self.Ay
        #     * self.params.cg_height
        #     * self.params.total_weight
        #     / ((self.params.trackwidth_f + self.params.trackwidth_r) / 2)
        # )

        # lat_wt_f = lat_wt * self.params.LLTD
        # lat_wt_r = lat_wt * (1 - self.params.LLTD)
        
        # NOTE: Left handed turn, so left tires are inner, right tires outer
        # weight transfer from inner tires to outer
        # w_fl = w_ftire - lat_wt_f
        # w_rl = w_rtire - lat_wt_r

        # w_fr = w_ftire + lat_wt_f
        # w_rr = w_rtire + lat_wt_r
        
        # Lazy zero-protection go crazy
        if(self.r == 0):
            self.alpha_f = 0
            self.alpha_r = 0
        else:
            self.alpha_f = self.beta + a * 1/self.r - self.delta
            self.alpha_r = self.beta - b * 1/self.r

        # 

        # self.fl_tire.Fz = w_fl
        # self.rl_tire.Fz = w_rl
        # self.fr_tire.Fz = w_fr
        # self.rr_tire.Fz = w_rr

        # self.fl_tire.epsilon = IA_fl
        # self.rl_tire.epsilon = IA_rl
        # self.fr_tire.epsilon = IA_fr
        # self.rr_tire.epsilon = IA_rr

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

