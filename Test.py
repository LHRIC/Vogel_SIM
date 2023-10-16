import state_models
import setups
import numpy as np
import matplotlib.pyplot as plt

state_in = state_models.StateInput(Ax=1, Ay=0.5, v=20, r=15, delta=0, beta=0)
setup = setups.VehicleSetup()

v = state_models.VehicleState(params=setup)
v.eval(state_in=state_in)

print(v.fl_tire.Fx, v.fr_tire.Fx, v.rl_tire.Fx, v.rr_tire.Fx)


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