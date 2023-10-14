import state_models
import models
import numpy as np
import matplotlib.pyplot as plt

import csv

DYN = models.DYN()
AERO = models.AERO()
PTN = models.PTN()


v = state_models.VehicleState(DYN=DYN, AERO=AERO, PTN=PTN, Ax=-1, Ay=1.5)
v.eval()





'''
with open('CharlesGundlachV.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Total Weight (lbs)", "Weight Distribution (rear)", "Fx_r (Both Tires)"])
    for total_weight in np.linspace(440, 660, 15):
        for wdf in np.linspace(0.2, 0.5, 15):
            DYN = models.DYN(overrides={
                "total_weight": total_weight,
                "weight_dist_f": wdf
            })
            AERO = models.AERO()
            PTN = models.PTN()

            v = state_models.VehicleState(DYN=DYN, AERO=AERO, PTN=PTN)
            Fx = v.eval()

            l = [total_weight, 1 - wdf, Fx*2]

            writer.writerow(l)
    f.close()
'''