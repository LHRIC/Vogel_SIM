import models
import ggv.GGV as GGV
import math

import matlab

class Vehicle:
    def __init__(self):
        self.AERO = models.AERO()
        self.DYN = models.DYN()
        self.PTN = models.PTN()

        self.gear_tot = self.PTN.gear_ratios[-1] * self.PTN.final_drive * self.PTN.primary_reduction
        self.v_max = self.PTN.shiftpoint / (self.gear_tot/self.DYN.tire_radius*60/(2 * math.pi))

        self.GGV = GGV.GGV(self.AERO, self.DYN, self.PTN, self.gear_tot, self.v_max)


if __name__ == "__main__":
    v = Vehicle()
    #print(len(v.PTN.rpm_range))
    #print(len(v.PTN.torque_curve))
    v.GGV.generate()
