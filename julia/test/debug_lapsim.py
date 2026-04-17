#!/usr/bin/env python3
"""
debug_lapsim.py
Run from repo root:  python julia/test/debug_lapsim.py

Prints the same key intermediate values as debug_lapsim.jl so the two
outputs can be compared line-by-line.
"""
import sys, pathlib, math
import numpy as np
from numpy.polynomial import Polynomial

PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2] / "python"
sys.path.insert(0, str(PYTHON_DIR))
import os; os.chdir(str(PYTHON_DIR))

import setups, state_models
from utilities import MF52
from Vehicle import Vehicle

params = setups.Panda()

# ── PARAMS ────────────────────────────────────────────────────────────────────
print("=== PARAMS ===")
print(f"  total_weight    = {params.total_weight:.4f} N")
print(f"  weight_dist_f   = {params.weight_dist_f:.4f}")
print(f"  cg_height       = {params.cg_height:.6f} m")
print(f"  trackwidth_f    = {params.trackwidth_f:.6f} m")
print(f"  trackwidth_r    = {params.trackwidth_r:.6f} m")
print(f"  tire_radius     = {params.tire_radius:.6f} m")
print(f"  Cl={params.Cl:.3f}  Cd={params.Cd:.3f}  CoP={params.CoP:.3f}")
print(f"  k_phi_f={params.k_phi_f:.4f}  k_phi_r={params.k_phi_r:.4f}")
print(f"  ride_rate_f={params.ride_rate_f:.1f}  ride_rate_r={params.ride_rate_r:.1f} N/mm")
print(f"  friction_x={params.friction_scaling_x:.3f}  friction_y={params.friction_scaling_y:.3f}")

# ── Build vehicle (loads trajectory + GGV) ────────────────────────────────────
vehicle = Vehicle(
    params=params,
    trajectory_path="./trajectory/23_michigan_endurance_ft.csv",
    is_closed=True,
)
g = vehicle.GGV
traj = vehicle.trajectory

print("\n=== TRAJECTORY ===")
print(f"  num_points = {traj.num_points}")
print(f"  radii[0:5] = {list(traj.radii[:5])}")
print(f"  radii min/max = {min(traj.radii):.4f} / {max(traj.radii):.4f} m")

gear_tot = params.gear_ratios[-1] * params.final_drive * params.primary_reduction
v_max    = params.shiftpoint / (gear_tot / params.tire_radius * 60 / (2 * math.pi))
print(f"\n=== GGV SETUP ===")
print(f"  gear_tot = {gear_tot:.6f}")
print(f"  v_max    = {v_max:.4f} m/s")

# ── GGV raw values ────────────────────────────────────────────────────────────
print("\n--- GGV raw accel values at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]:
    grip = g.calc_grip_lim_max_accel(v)
    Fx_w, gear_idx = g.calc_power_lim_max_accel(max(7.5, v))
    power = (Fx_w - params.Cd * v**2) / params.total_weight
    accel = min(grip, power)
    print(f"  v={v:5.1f} m/s | grip={grip:.4f} g | power={power:.4f} g | accel={accel:.4f} g | gear={gear_idx}")

print("\n--- GGV raw braking values at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]:
    decel = g.calc_decel(v)
    print(f"  v={v:5.1f} m/s | decel={decel:.4f} g")

print("\n--- Lateral envelope (precomputed) ---")
for i, R in enumerate(g.radii_range):
    lat_g = g.lateral_capability[i]   # still a list before generate()
    v_corn = math.sqrt(lat_g * 9.81 * R)
    print(f"  R={R:5.2f} m | lat={lat_g:.4f} g | v_corner={v_corn:.4f} m/s")

# ── Generate GGV ──────────────────────────────────────────────────────────────
print("\n=== Running generate() ===")
g._calc_lateral = True
g.generate()

print("\n--- Fitted accel_capability at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]:
    print(f"  accel_capability({v:.1f}) = {g.accel_capability.evaluate(v):.6f} g")

print("\n--- Fitted braking_capability at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]:
    print(f"  braking_capability({v:.1f}) = {g.braking_capability.evaluate(v):.6f} g")

print("\n--- Fitted cornering_capability at selected radii ---")
for R in [5.0, 8.0, 12.0, 20.0, 30.0]:
    print(f"  cornering_capability({R:.1f} m) = {g.cornering_capability.evaluate(R):.4f} m/s")

print("\n--- Fitted lateral_capability at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]:
    print(f"  lateral_capability({v:.1f} m/s) = {g.lateral_capability.evaluate(v):.6f} g")

# ── Forward pass debug (first 5 segments) ────────────────────────────────────
print("\n=== FORWARD PASS (first 5 trajectory segments) ===")
vel = 20.0 * 0.3048
for pi in range(min(5, traj.num_points - 1)):
    x1 = traj.points[0][pi]; y1 = traj.points[1][pi]
    x2 = traj.points[0][pi+1]; y2 = traj.points[1][pi+1]
    seg = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    r = traj.radii[pi]
    v_max_seg = min(v_max, g.cornering_capability.evaluate(r))
    AX_cap = g.accel_capability.evaluate(vel)
    AY_cap = g.lateral_capability.evaluate(vel)
    AY_act = vel**2 / (r * 9.81)
    ax_pot = AX_cap * math.sqrt(1 - (min(AY_cap, AY_act)/AY_cap)**2)
    ddelta = seg / 10
    p = Polynomial([-ddelta, vel, 0.5*9.81*ax_pot])
    dt = max(p.roots())
    dv = 9.81 * ax_pot * dt
    print(f"seg {pi} | seg={seg:.4f}m r={r:.2f}m | vel={vel:.4f}→{min(v_max_seg, vel+dv):.4f} m/s | "
          f"AX={AX_cap:.4f} AY_cap={AY_cap:.4f} AY_act={AY_act:.4f} | ax_pot={ax_pot:.4f} | dt={dt:.6f}s")
    vel = min(v_max_seg, vel + dv)

# ── Full simulation ───────────────────────────────────────────────────────────
vehicle.simulate_forwards(20.0 * 0.3048)
print("\n--- Forward pass summary ---")
print(f"  max velocity_f = {max(vehicle.velocity_f):.4f} m/s")
print(f"  min velocity_f = {min(vehicle.velocity_f):.4f} m/s")
print(f"  total fwd time = {max(vehicle.time):.4f} s")
print(f"  velocity_f[0:5]    = {[round(x,4) for x in vehicle.velocity_f[:5]]} m/s")
print(f"  velocity_f[-5:]    = {[round(x,4) for x in vehicle.velocity_f[-5:]]} m/s")

vehicle.simulate_reverse()
print("\n--- Reverse pass summary ---")
print(f"  max velocity_r = {max(vehicle.velocity_r):.4f} m/s")
print(f"  min velocity_r = {min(vehicle.velocity_r):.4f} m/s")
print(f"  velocity_r[0:5]    = {[round(x,4) for x in vehicle.velocity_r[:5]]} m/s")
print(f"  velocity_r[-5:]    = {[round(x,4) for x in vehicle.velocity_r[-5:]]} m/s")

vehicle.simulate_forwards(vehicle.velocity_r[0])

# Merge
merged_vel = np.where(vehicle.velocity_f < vehicle.velocity_r, vehicle.velocity_f, vehicle.velocity_r)
n_fwd = np.sum(vehicle.velocity_f <= vehicle.velocity_r)
n_rev = len(merged_vel) - n_fwd
print("\n--- Merged velocity summary ---")
print(f"  max velocity   = {max(merged_vel):.4f} m/s")
print(f"  min velocity   = {min(merged_vel):.4f} m/s")
print(f"  fwd-pass wins  = {n_fwd} points ({100*n_fwd/len(merged_vel):.1f}%)")
print(f"  rev-pass wins  = {n_rev} points ({100*n_rev/len(merged_vel):.1f}%)")

laptime = vehicle.simulate_endurance()
print(f"\n=== RESULT ===")
print(f"  laptime = {laptime:.4f} s")
