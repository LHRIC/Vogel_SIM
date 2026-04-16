"""
test_ggv_components.py
======================
Verifies that individual GGV components (MF52 tire forces, weight-transfer normal
loads, grip-limited / power-limited / braking acceleration arrays) agree between
Python and Julia to within 0.1%.

Usage (from repo root):
    python julia/test/test_ggv_components.py
"""

import os, sys, pathlib, json, subprocess, tempfile

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
VENV_PY   = REPO_ROOT / ".venv" / "bin" / "python3"
PYTHON    = str(VENV_PY) if VENV_PY.exists() else sys.executable
JULIA_DIR = REPO_ROOT / "julia"
TOL       = 0.001   # 0.1 %


# ── Python reference data ────────────────────────────────────────────────────

def get_python_data() -> dict:
    code = r"""
import sys, pathlib, json
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
import os; os.chdir(sys.path[0])

import numpy as np
import setups, state_models
from utilities import MF52
from Vehicle import Vehicle

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
"""
    tmp = JULIA_DIR / "test" / "_py_ggv_data.py"
    tmp.write_text(code)
    r = subprocess.run([PYTHON, str(tmp)], capture_output=True, text=True, cwd=str(REPO_ROOT))
    tmp.unlink()
    if r.returncode != 0:
        raise RuntimeError(f"Python data extraction failed:\n{r.stderr[-3000:]}")
    for line in r.stdout.splitlines():
        if line.startswith("JSON_DATA="):
            return json.loads(line[len("JSON_DATA="):])
    raise RuntimeError(f"No JSON_DATA in output:\n{r.stdout[-1000:]}")


# ── Julia reference data ─────────────────────────────────────────────────────

def get_julia_data() -> dict:
    code = r"""
using Pkg; Pkg.activate(joinpath(@__DIR__, ".."))
include(joinpath(@__DIR__, "..", "src", "VogelSIM.jl"))
using .VogelSIM

base_dir = joinpath(@__DIR__, "..", "..")
params   = Panda(base_dir)
mf52     = MF52(base_dir)

Fx_ref = mf52_Fx(mf52, 800.0, 0.05, 0.0)
Fy_ref = mf52_Fy(mf52, 800.0, 0.1,  0.0)

vs = VehicleState(mf52, params)
eval!(vs, StateInput(Ax=0.5, Ay=1.0, v=20.0, r=10.0, delta=0.05, beta=0.0))

gear_tot = params.gear_ratios[end]*params.final_drive*params.primary_reduction
v_max    = params.shiftpoint/(gear_tot/params.tire_radius*60/(2π))
g        = GGV(params, mf52, gear_tot, v_max; calc_lateral=false)

vrange = Float64.(collect(g.velocity_range))
grip   = [calc_grip_lim_max_accel(g, v) for v in vrange]
power_v = Float64[]
for v in vrange
    Fx_r, _ = calc_power_lim_max_accel(g, max(7.5, v))
    Fx_r -= g.params.Cd*v^2
    push!(power_v, Fx_r/g.params.total_weight)
end
braking = [calc_decel(g, v) for v in vrange]

println("SCALAR Fx_ref=$(Fx_ref)")
println("SCALAR Fy_ref=$(Fy_ref)")
println("SCALAR Fz_FL=$(vs.fl_tire.Fz)")
println("SCALAR Fz_FR=$(vs.fr_tire.Fz)")
println("SCALAR Fz_RL=$(vs.rl_tire.Fz)")
println("SCALAR Fz_RR=$(vs.rr_tire.Fz)")
println("ARRAY grip="    * join(grip,    ","))
println("ARRAY power="   * join(power_v, ","))
println("ARRAY braking=" * join(braking, ","))
"""
    tmp = JULIA_DIR / "test" / "_jl_ggv_data.jl"
    tmp.write_text(code)
    r = subprocess.run(["julia", str(tmp)], capture_output=True, text=True, cwd=str(REPO_ROOT))
    tmp.unlink()
    if r.returncode != 0:
        raise RuntimeError(f"Julia data extraction failed:\n{r.stderr[-3000:]}")

    scalars, arrays = {}, {}
    for line in r.stdout.splitlines():
        if line.startswith("SCALAR "):
            k, v = line[7:].split("=", 1)
            scalars[k] = float(v)
        elif line.startswith("ARRAY "):
            k, v = line[6:].split("=", 1)
            arrays[k] = [float(x) for x in v.split(",")]

    if not scalars:
        raise RuntimeError(f"No data in Julia output:\n{r.stdout[-1000:]}")

    return {
        "Fx_ref": scalars["Fx_ref"], "Fy_ref": scalars["Fy_ref"],
        "loads": {"FL": scalars["Fz_FL"], "FR": scalars["Fz_FR"],
                  "RL": scalars["Fz_RL"], "RR": scalars["Fz_RR"]},
        "grip": arrays["grip"], "power": arrays["power"],
        "braking": arrays["braking"],
    }


# ── Comparison helpers ───────────────────────────────────────────────────────

def rel_err(a, b):
    if abs(a) < 1e-10:
        return abs(b - a)
    return abs(b - a) / abs(a)


def check_scalar(name, py_val, jl_val, tol=TOL):
    err = rel_err(py_val, jl_val)
    status = "PASS" if err <= tol else "FAIL"
    print(f"  {status}  {name:30s}  py={py_val:12.6f}  jl={jl_val:12.6f}  err={err*100:.4f}%")
    assert err <= tol, f"{name}: rel_err={err*100:.4f}% > {tol*100:.1f}%"


def check_array(name, py_arr, jl_arr, tol=TOL):
    assert len(py_arr) == len(jl_arr), f"{name}: length mismatch {len(py_arr)} vs {len(jl_arr)}"
    max_err = 0.0
    for i, (a, b) in enumerate(zip(py_arr, jl_arr)):
        e = rel_err(a, b)
        max_err = max(max_err, e)
        if e > tol:
            print(f"  FAIL  {name}[{i}]: py={a:.6f}  jl={b:.6f}  err={e*100:.4f}%")
    assert max_err <= tol, f"{name}: max rel_err={max_err*100:.4f}% > {tol*100:.1f}%"
    print(f"  PASS  {name:30s}  max_err={max_err*100:.4f}%")


# ── Test ─────────────────────────────────────────────────────────────────────

def test_ggv_components():
    print("\nCollecting Python reference data …")
    py = get_python_data()

    print("Collecting Julia reference data …")
    jl = get_julia_data()

    print("\n--- MF52 tire model ---")
    check_scalar("Fx(800N, kappa=0.05, 0°)", py["Fx_ref"], jl["Fx_ref"])
    check_scalar("Fy(800N, 0.1rad, 0°)",     py["Fy_ref"], jl["Fy_ref"])

    print("\n--- Weight transfer (Ax=0.5g, Ay=1g, v=20m/s, r=10m) ---")
    for corner in ("FL", "FR", "RL", "RR"):
        check_scalar(f"Fz_{corner} (N)", py["loads"][corner], jl["loads"][corner])

    print("\n--- GGV raw arrays ---")
    check_array("grip_lim_accel", py["grip"],    jl["grip"])
    check_array("power_lim_accel", py["power"],   jl["power"])
    check_array("braking_accel",   py["braking"], jl["braking"])

    print("\nAll GGV component tests PASSED.")


if __name__ == "__main__":
    test_ggv_components()
