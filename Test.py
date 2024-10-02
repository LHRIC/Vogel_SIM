import state_models
import setups
import numpy as np
import math
import matplotlib.pyplot as plt


Fz_f = []
Fz_r = []

for vel in np.linspace(10, 23, 13):
    # for a vehicle in this, give me give me vehicle outputs
    # when driving across a track, calculate these variables and calculate Fz's from it
    # Ax, Ay, v, r, 
    state_in = state_models.StateInput(Ax=-1.5, Ay=0, v=vel, r=0, delta=0, beta=0)
    setup = setups.VehicleSetup()

    v = state_models.VehicleState(params=setup)
    v.eval(state_in=state_in)

    Fz_f.append(v.fl_tire.Fz)
    Fz_r.append(v.rl_tire.Fz)

print(Fz_f)
print(Fz_r)



state_in = state_models.StateInput(Ax=0, Ay=1.7, v=20, r=0, delta=0, beta=0)
setup = setups.VehicleSetup(overrides={"torsional_rigidity": 1300})
print(setup.LLTD)



v = state_models.VehicleState(params=setup)
v.eval(state_in=state_in)

print(v.fl_tire.Fz, v.fr_tire.Fz)
print(v.rl_tire.Fz, v.rr_tire.Fz)

print(v.fl_sus_dz, v.fr_sus_dz)
print(v.rl_sus_dz, v.rr_sus_dz)

print(math.degrees(v.fl_tire.epsilon), math.degrees(v.fr_tire.epsilon))
print(math.degrees(v.rl_tire.epsilon), math.degrees(v.rr_tire.epsilon))
