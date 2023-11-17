import models
import math
import numpy as np
import csv
import pathlib
import os

class VehicleSetup():
    def __init__(self, overrides={}):
        cur_path = pathlib.Path(__file__).parent.resolve()

        self.total_weight = 650

        # This means that 48% of the weight is on the fronts
        # Drill that into your head you fucking idiot
        self.weight_dist_f = 0.46

        self.cg_height = 12.1 # inches
        self.wheelbase = 1.535  # meters
        self.trackwidth_f = 48.875  # inches
        self.trackwidth_r = self.trackwidth_f # inches

        self.torsional_rigidity = 1184 #Nm/deg
        self.tr_nom = 1184 #Nm/deg

        self.roll_grad_f = math.radians(0.75) #(rad/g)
        self.roll_grad_r = math.radians(0.75)
        self.pitch_grad = math.radians(0.5)

        self.ride_rate_f = 33887.043 # N/m
        self.ride_rate_r = 33887.043

        self.tire_radius = 0.2032 # meters

        self.static_camber_f = 0 # radians
        self.static_camber_r = 0
        self.camber_comp_f = 0
        self.camber_comp_r = 0

        self.camber_gain_p = 1

        self.caster_f = 0
        self.caster_r = 4.1568

        self.KPI_f = 0
        self.KPI_r = 0

        self.friction_scaling_x = 0.6
        self.friction_scaling_y = 0.6

        self.Cl = 4 #ClA
        self.Cd = 1.5
        #That mean x% of df on the fronts
        self.CoP = self.weight_dist_f + 0.05

        self.torque_mod = 1


        self.rpm_range = []
        self.torque_curve = []
        with open(os.path.join(cur_path, "23_torque_curve.csv")) as torque_curve_infile:
            spamreader = csv.reader(torque_curve_infile)
            for row in spamreader:
                self.rpm_range.append(float(row[0]))
                self.torque_curve.append(float(row[1]))

        self.rpm_range = np.array(self.rpm_range)
        self.torque_curve = np.array(self.torque_curve)        
        
        self.primary_reduction = 76/36
        self.gear_ratios = [33/12, 32/16, 30/18, 26/18, 30/23, 29/24]
        self.num_gears = len(self.gear_ratios)
        self.final_drive = 37/11
        self.shiftpoint = 12500
        self.drivetrain_losses = 0.85
        self.shift_time = 0.25 #seconds
        self.diff_locked = False; #False - open, True - closed
    
        for key in overrides.keys():
            val = overrides[key]

            try:
                setattr(self, key, val)
            except:
                exit("Text 508-333-3888, this error should not be thrown.")
        self.compute_deriv_params()
        self.convert_units()
        self.LLTD = self.calc_lltd(self.torsional_rigidity)
    
    def convert_units(self):
        w_mod = (self.torsional_rigidity - self.tr_nom) / 47.5 * 9.81 #N
        self.total_weight = self.total_weight * 4.4482216153 + w_mod# N
        self.total_weight_f = self.total_weight * self.weight_dist_f
        self.total_weight_r = self.total_weight * (1 - self.weight_dist_f)

        self.cg_height = self.cg_height / 39.37  # meters
        self.trackwidth_f = self.trackwidth_f * 0.0254  # meters
        self.trackwidth_r = self.trackwidth_r * 0.0254  # meters
        self.trackwidth_max = max(self.trackwidth_f, self.trackwidth_r)

        self.camber_comp_f = self.camber_comp_f / 100
        self.camber_comp_r = self.camber_comp_r / 100

        self.camber_gain_f = self.camber_roll_induced_f * self.camber_comp_f
        self.camber_gain_r = self.camber_roll_induced_r * self.camber_comp_r
    
    def compute_deriv_params(self):
        self.camber_roll_induced_f = math.asin(2 / self.trackwidth_f)
        self.camber_roll_induced_r = math.asin(2 / self.trackwidth_r)
    
    def calc_lltd(self, k_ch):
        k_phi_f = 486 # nm/deg
        k_phi_r = 650 #nm/deg

        Z_f = 0.02785
        Z_r = 0.03556

        ds_f = self.cg_height - Z_f
        ds_r = self.cg_height - Z_r

        m_s_f = self.total_weight_f / 9.81 - 10
        m_s_r = self.total_weight_r / 9.81 - 10

        _lambda = k_phi_f / (k_phi_f + k_phi_r)
        mu = k_ch / (k_phi_f + k_phi_r)

        A1 = (_lambda**2 - (mu + 1) * _lambda) / (_lambda**2 - _lambda - mu)
        A2 = (ds_f * m_s_f) / (self.cg_height * self.total_weight / 9.81)
        B1 = (mu*_lambda)/(_lambda**2 - _lambda - mu)
        B2 = (ds_r * m_s_r) / (self.cg_height * self.total_weight / 9.81)
        C1 = (Z_f * m_s_f) / (self.cg_height * self.total_weight / 9.81)
        D1 = (0.205 * 10) / (self.cg_height * self.total_weight / 9.81)

        return A1*A2 - B1*B2 + C1+ D1

if __name__ == "__main__":
    v = VehicleSetup()