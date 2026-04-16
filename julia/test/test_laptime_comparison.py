"""
test_laptime_comparison.py
==========================
Runs both the Python and Julia laptime simulations and asserts that the
1-lap endurance time agrees to within 0.1%.

Usage (from repo root):
    python julia/test/test_laptime_comparison.py

The script auto-discovers the repo root from its own location and uses the
project venv at .venv/bin/python3 (if present) or the current interpreter.
"""

import os
import sys
import subprocess
import pathlib

REPO_ROOT  = pathlib.Path(__file__).resolve().parents[2]
JULIA_DIR  = REPO_ROOT / "julia"
VENV_PY    = REPO_ROOT / ".venv" / "bin" / "python3"
PYTHON     = str(VENV_PY) if VENV_PY.exists() else sys.executable
TOLERANCE  = 0.001   # 0.1 %


# ── Python simulation ────────────────────────────────────────────────────────

def run_python() -> float:
    """Return 1-lap endurance time from the Python simulation."""
    code = r"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
import os; os.chdir(sys.path[0])

import setups
from Vehicle import Vehicle

params  = setups.Panda()
vehicle = Vehicle(params=params,
                  trajectory_path='./trajectory/23_michigan_endurance_ft.csv',
                  is_closed=True)
vehicle.GGV._calc_lateral = True
vehicle.GGV.generate()
laptime = vehicle.simulate_endurance()
print(f'LAPTIME={laptime:.10f}')
"""
    script = JULIA_DIR / "test" / "_run_python_sim.py"
    script.write_text(code)

    result = subprocess.run(
        [PYTHON, str(script)],
        capture_output=True, text=True,
        cwd=str(REPO_ROOT)
    )
    script.unlink()

    if result.returncode != 0:
        raise RuntimeError(f"Python simulation failed:\n{result.stderr[-2000:]}")

    for line in result.stdout.splitlines():
        if line.startswith("LAPTIME="):
            return float(line.split("=")[1])

    raise RuntimeError(f"No LAPTIME in Python output:\n{result.stdout[-1000:]}")


# ── Julia simulation ─────────────────────────────────────────────────────────

def run_julia() -> float:
    """Return 1-lap endurance time from the Julia simulation."""
    julia_code = r"""
using Pkg; Pkg.activate(joinpath(@__DIR__, ".."))
include(joinpath(@__DIR__, "..", "src", "VogelSIM.jl"))
using .VogelSIM

base_dir   = joinpath(@__DIR__, "..", "..")
trajectory = joinpath(base_dir, "trajectory", "23_michigan_endurance_ft.csv")
params  = Panda(base_dir)
vehicle = Vehicle(params, trajectory, true; base_dir=base_dir)
generate!(vehicle.ggv)
laptime = simulate_endurance!(vehicle)
println("LAPTIME=$(laptime)")
"""
    script = JULIA_DIR / "test" / "_run_julia_sim.jl"
    script.write_text(julia_code)

    result = subprocess.run(
        ["julia", str(script)],
        capture_output=True, text=True,
        cwd=str(REPO_ROOT)
    )
    script.unlink()

    if result.returncode != 0:
        raise RuntimeError(f"Julia simulation failed:\n{result.stderr[-2000:]}")

    for line in result.stdout.splitlines():
        if line.startswith("LAPTIME="):
            return float(line.split("=")[1])

    raise RuntimeError(f"No LAPTIME in Julia output:\n{result.stdout[-1000:]}")


# ── Test ─────────────────────────────────────────────────────────────────────

def test_laptime_within_tolerance():
    print("Running Python simulation …")
    t_py = run_python()
    print(f"  Python  1-lap time: {t_py:.4f} s")

    print("Running Julia simulation …")
    t_jl = run_julia()
    print(f"  Julia   1-lap time: {t_jl:.4f} s")

    rel_err = abs(t_py - t_jl) / t_py
    print(f"  Relative error:     {rel_err*100:.4f}%  (tolerance {TOLERANCE*100:.1f}%)")

    assert rel_err <= TOLERANCE, (
        f"Laptime mismatch exceeds {TOLERANCE*100:.1f}%: "
        f"Python={t_py:.6f} s, Julia={t_jl:.6f} s, "
        f"rel_err={rel_err*100:.4f}%"
    )
    print("PASS — laptimes agree within tolerance.")


if __name__ == "__main__":
    test_laptime_within_tolerance()
