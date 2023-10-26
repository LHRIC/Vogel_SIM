import state_models
import setups
import numpy as np
import matplotlib.pyplot as plt

'''
Fz_f = []
Fz_r = []

for vel in np.linspace(10, 23, 13):
    state_in = state_models.StateInput(Ax=-1.5, Ay=0, v=vel, r=0, delta=0, beta=0)
    setup = setups.VehicleSetup()

    v = state_models.VehicleState(params=setup)
    v.eval(state_in=state_in)

    Fz_f.append(v.fl_tire.Fz)
    Fz_r.append(v.rl_tire.Fz)

print(Fz_f)
print(Fz_r)

'''

state_in = state_models.StateInput(Ax=0, Ay=2.1, v=23, r=0, delta=0, beta=0)
setup = setups.VehicleSetup()

v = state_models.VehicleState(params=setup)
v.eval(state_in=state_in)

print(v.fl_tire.Fz, v.rl_tire.Fz, v.fr_tire.Fz, v.rr_tire.Fz)