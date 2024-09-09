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
        dFz_aero_f = 1.293/2*self.params.CoP*self.params.Cl*self.v**2
        dFz_aero_r = 1.293/2*(1-self.params.CoP)*self.params.Cl*self.v**2
        self.downforce = dFz_aero_f + dFz_aero_r

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
        phi_r = math.degrees(math.asin((dz_RR-dz_RL)/(self.params.trackwidth_r*1000)))
        self.phi = (phi_f+phi_r)/2 # If they're not the same check if ride rate and roll stiffness correlate #TODO add ARBs
        
        # Inclination angle
        IA_FL = camber_FL - self.phi
        IA_FR = camber_FR + self.phi
        IA_RL = camber_RL - self.phi
        IA_RR = camber_RR + self.phi

        # Body pitch (degrees)
        self.theta = math.degrees(math.asin(((dz_FR+dz_FL)/2 - (dz_RR+dz_RL)/2)/(self.params.wheelbase*1000)))

        # CG Position
        self.dz_CG = (((dz_FR+dz_FL)/2)-((dz_RR+dz_RL)/2))*self.params.weight_dist_f+((dz_RR+dz_RL)/2) # delta cg (mm, -Z is +)
        self.cgz = -self.dz_CG/1000 + self.params.cg_height # cg height (m)

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

        # Define wheelbase aliases based on CG location
        a = self.params.wheelbase * (1 - self.params.weight_dist_f)
        b = self.params.wheelbase * self.params.weight_dist_f

        # Lazy zero-protection go crazy
        if(self.r == 0):
            self.alpha_f = 0
            self.alpha_r = 0
        else:
            self.alpha_f = self.beta + a * 1/self.r - self.delta
            self.alpha_r = self.beta - b * 1/self.r

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

