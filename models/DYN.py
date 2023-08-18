import math
from .SystemModel import SystemModel

class DYN(SystemModel):
    def __init__(self):
        self.LLTD = 0.5
        self.driver_weight = 96
        self.vehicle_weight = 400
        self.total_weight = self.driver_weight + self.vehicle_weight

        self.weight_dist_f = 0.45
        self.cg_height = 12.5  # inches
        self.wheelbase = 1.535  # meters
        self.trackwidth_f = 48  # inches
        self.trackwidth_r = 48  # inches

        self.roll_grad_f = 0 #(rad/g)
        self.roll_grad_r = 0
        self.pitch_grad = 0
        self.ride_rate_f = 31522.830300000003 # N/m
        self.ride_rate_r = 31522.830300000003

        self.tire_radius = 0.2032 # meters

        self.static_camber_f = 0 # radians
        self.static_camber_r = 0
        self.camber_comp_f = 0
        self.camber_comp_r = 0

        self.caster_f = 0
        self.caster_r = 4.1568

        self.KPI_f = 0
        self.KPI_r = 0

        self.camber_roll_induced_f = math.asin(2 / self.trackwidth_f)
        self.camber_roll_induced_r = math.asin(2 / self.trackwidth_r)

        self.friction_scaling_x = 0.6
        self.friction_scaling_y = 0.6

        #Unit Conversions

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