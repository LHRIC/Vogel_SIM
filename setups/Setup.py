import models
import math
import numpy as np

class VehicleSetup():
    def __init__(self, overrides={}):
        self.LLTD = 0.5
        self.total_weight = 496

        # This means that 45% of the weight is on the fronts
        # Drill that into your head you fucking idiot
        self.weight_dist_f = 0.45

        self.cg_height = 12.5  # inches
        self.wheelbase = 1.535  # meters
        self.trackwidth_f = 48  # inches
        self.trackwidth_r = self.trackwidth_f # inches

        self.roll_grad_f = math.radians(0.75) #(rad/g)
        self.roll_grad_r = math.radians(0.75)
        self.pitch_grad = math.radians(0.5)

        self.ride_rate_f = 31522.8303 # N/m
        self.ride_rate_r = 31522.8303

        self.tire_radius = 0.2032 # meters

        self.static_camber_f = 0 # radians
        self.static_camber_r = 0
        self.camber_comp_f = 0
        self.camber_comp_r = 0

        self.caster_f = 0
        self.caster_r = 4.1568

        self.KPI_f = 0
        self.KPI_r = 0

        self.friction_scaling_x = 0.6
        self.friction_scaling_y = 0.6

        self.Cl = 1.77 #ClA
        self.Cd = 0.8
        self.CoP = 0.48

        self.torque_mod = 1
        self.rpm_range = np.array([*range(6200, 14100 + 100, 100)])
        self.torque_curve = self.torque_mod * np.array([41.57, 42.98, 44.43, 45.65, 46.44, 47.09, 47.52, 48.58, 49.57, 50.41, 51.43, 51.48, 51, 49.311, 48.94, 48.66, 49.62, 49.60, 47.89, 47.91, 48.09, 48.57, 49.07, 49.31, 49.58, 49.56, 49.84, 50.10, 50.00, 50.00, 50.75, 51.25, 52.01, 52.44, 52.59, 52.73, 53.34, 53.72,
                             52.11, 52.25, 51.66, 50.5, 50.34, 50.50, 50.50, 50.55, 50.63, 50.17, 50.80, 49.73, 49.35, 49.11, 48.65, 48.28, 48.28, 47.99, 47.68, 47.43, 47.07, 46.67, 45.49, 45.37, 44.67, 43.8, 43.0, 42.3, 42.00, 41.96, 41.70, 40.43, 39.83, 38.60, 38.46, 37.56, 36.34, 35.35, 33.75, 33.54, 32.63, 31.63])
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
    
    def convert_units(self):
        self.total_weight = self.total_weight * 4.4482216153  # N
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