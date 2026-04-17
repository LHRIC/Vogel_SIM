
import sys, pathlib, json
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
import os; os.chdir(sys.path[0])

import numpy as np
import python.setups as setups, python.state_models as state_models
from python.utilities import MF52
from python.Vehicle import Vehicle

# --- MF52 tire forces at reference conditions ---
tm = MF52()
Fx_ref = float(tm.Fx(800.0, 0.05, 0.0))
Fy_ref = float(tm.Fy(800.0, 0.1,  0.0))   # 0.1 rad slip angle

# --- Weight transfer at reference state (1g lat, 0.5g accel) ---
params  = setups.Panda()
vehicle = Vehicle(params=params,
                  trajectory_path='./trajectory/23_michigan_endurance_ft.csv',
                  is_closed=True)
g = vehicle.GGV
vs = state_models.VehicleState(params=params)
si = state_models.StateInput(Ax=0.5, Ay=1.0, v=20.0, r=10.0, delta=0.05, beta=0.0)
vs.eval(state_in=si)
loads = {
    'FL': float(vs.fl_tire.Fz), 'FR': float(vs.fr_tire.Fz),
    'RL': float(vs.rl_tire.Fz), 'RR': float(vs.rr_tire.Fz),
}

# --- GGV raw arrays (before fitting) ---
g._calc_lateral = False
vrange = list(g.velocity_range)
grip, power, braking = [], [], []
for v in vrange:
    grip.append(float(g.calc_grip_lim_max_accel(v)))
    Fx_r, _ = g.calc_power_lim_max_accel(max(7.5, float(v)))
    Fx_r -= params.Cd * float(v)**2
    power.append(float(Fx_r / params.total_weight))
    braking.append(float(g.calc_decel(float(v))))

data = {
    'Fx_ref': Fx_ref, 'Fy_ref': Fy_ref,
    'loads': loads,
    'vrange': [float(v) for v in vrange],
    'grip': grip, 'power': power, 'braking': braking,
}
print("JSON_DATA=" + json.dumps(data))
