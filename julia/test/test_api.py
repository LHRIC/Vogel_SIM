"""
test_api.py
===========
Integration tests for the VogelSIM HTTP API (server.jl).

Requires a running server.  Set the SERVER_URL environment variable to
override the default (http://localhost:8082).

Usage:
    # start the server in one terminal (from the julia/ directory):
    #   julia server.jl
    # then in another:
    python julia/test/test_api.py

Exit code 0 = all tests passed.
"""

import csv
import json
import os
import pathlib
import sys
import urllib.request
import urllib.error

import scipy.io

BASE_URL = os.environ.get("SERVER_URL", "http://localhost:8082").rstrip("/")

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"

_failures = []

# ── Fixture paths ─────────────────────────────────────────────────────────────

_REPO = pathlib.Path(__file__).resolve().parents[2]
_PY   = _REPO / "python"

_TORQUE_CSV    = _PY / "setups"    / "23_torque_curve.csv"
_XY_CSV        = _PY / "trajectory" / "23_michigan_endurance_ft.csv"
_RADII_CSV     = _PY / "trajectory" / "23_michigan_endurance_radii_m.csv"
_CURVATURE_CSV = _PY / "trajectory" / "23_michigan_endurance_curvature_m.csv"
_FX_MAT        = _PY / "utilities"  / "18.0x6.0-10_R20_DriveBrakeComb.mat"
_FY_MAT        = _PY / "utilities"  / "16x7.5-10_R20_Cornering.mat"


# ── Fixture loaders ───────────────────────────────────────────────────────────

def _load_torque_curve():
    rpm, torque = [], []
    with open(_TORQUE_CSV) as f:
        for row in csv.reader(f):
            rpm.append(float(row[0]))
            torque.append(float(row[1]))
    return {"rpm": rpm, "torque": torque, "throttle": None}


def _load_trajectory():
    def _col(path, col_name):
        with open(path) as f:
            reader = csv.DictReader(f)
            return [float(r[col_name]) for r in reader]

    FT_TO_M = 0.3048
    xy = []
    with open(_XY_CSV) as f:
        for row in csv.DictReader(f):
            xy.append((float(row["X"]) * FT_TO_M, float(row["Y"]) * FT_TO_M))
    x = [p[0] for p in xy]
    y = [p[1] for p in xy]

    radii     = _col(_RADII_CSV,     "radii")
    curvature = _col(_CURVATURE_CSV, "curvature")

    return {"x": x, "y": y, "radii": radii, "curvature": curvature}


def _load_mf52():
    fx = scipy.io.loadmat(str(_FX_MAT))
    fy = scipy.io.loadmat(str(_FY_MAT))
    return {
        "fx_params": list(float(v) for v in fx["x0"].flatten()),
        "fy_params": list(float(v) for v in fy["x0"].flatten()),
    }


def _panda_body(**overrides):
    """Return a complete /lapsim request body using Panda defaults."""
    body = {
        # Params fields (pre-unit-conversion values matching Panda())
        "total_weight":        595.0,
        "weight_dist_f":       0.46,
        "cg_height":           12.0866,
        "wheelbase":           1.535,
        "trackwidth_f":        48.875,
        "trackwidth_r":        48.875,
        "rollc_f":             0.028,
        "rollc_r":             0.036,
        "asquat":              0.0,
        "adive":               0.0,
        "torsional_rigidity":  1184.0,
        "tr_nom":              1184.0,
        "ride_rate_f":         31430.0,
        "ride_rate_r":         38375.0,
        "k_phi_f":             486.0,
        "k_phi_r":             650.0,
        "tire_radius":         0.2032,
        "static_camber_f":     0.0,
        "static_camber_r":     0.0,
        "camber_gain_f":       0.0,
        "camber_gain_r":       0.0,
        "friction_scaling_x":  0.6,
        "friction_scaling_y":  0.6,
        "Cl":                  2.8,
        "Cd":                  1.1,
        "CoP":                 0.51,
        "primary_reduction":   38.0 / 18.0,
        "gear_ratios":         [33/12, 32/16, 30/18, 26/18, 30/23, 29/24],
        "num_gears":           6,
        "final_drive":         37.0 / 11.0,
        "shiftpoint":          12500.0,
        "drivetrain_losses":   0.85,
        "shift_time":          0.25,
        "diff_locked":         False,
        "LLTD":                0.5,
        # Composite fields (loaded from files)
        "torque_curve": _load_torque_curve(),
        "mf52":         _load_mf52(),
        "trajectory":   _load_trajectory(),
    }
    body.update(overrides)
    return body


# ── HTTP helpers ──────────────────────────────────────────────────────────────

def _get(path: str):
    with urllib.request.urlopen(BASE_URL + path, timeout=30) as resp:
        return resp.status, resp.read().decode()


def _post(path: str, body: dict):
    data = json.dumps(body).encode()
    req  = urllib.request.Request(
        BASE_URL + path, data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=300) as resp:
            return resp.status, json.loads(resp.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode())


# ── Assertion helpers ─────────────────────────────────────────────────────────

def check(name: str, cond: bool, detail: str = ""):
    if cond:
        print(f"  {PASS}  {name}")
    else:
        msg = name + (f": {detail}" if detail else "")
        print(f"  {FAIL}  {msg}")
        _failures.append(msg)


def check_eq(name: str, got, expected):
    check(name, got == expected, f"got {got!r}, expected {expected!r}")


def check_in_range(name: str, value: float, lo: float, hi: float):
    check(name, lo <= value <= hi, f"{value} not in [{lo}, {hi}]")


def check_keys(name: str, d: dict, *keys):
    missing = [k for k in keys if k not in d]
    check(name, not missing, f"missing keys: {missing}")


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_root():
    print("\n── GET / ──────────────────────────────────────────────────────────")
    status, body = _get("/")
    check_eq("status 200", status, 200)
    check("non-empty body", len(body) > 0)


def test_health():
    print("\n── GET /health ────────────────────────────────────────────────────")
    status, body = _get("/health")
    data = json.loads(body)
    check_eq("status 200", status, 200)
    check_keys("response keys", data, "status", "timestamp", "julia_version")
    check_eq("status=ok", data["status"], "ok")
    check("julia_version present", len(data["julia_version"]) > 0)


def test_lapsim_args():
    print("\n── GET /lapsim/args ───────────────────────────────────────────────")
    status, body = _get("/lapsim/args")
    data = json.loads(body)
    check_eq("status 200", status, 200)
    check("returns list", isinstance(data, list))
    check("non-empty", len(data) > 0)

    names = {entry["name"] for entry in data}
    for expected in ("total_weight", "Cl", "Cd", "torque_curve",
                     "trajectory", "mf52", "mesh_resolution"):
        check(f"arg '{expected}' present", expected in names)

    # torque_curve entry should have a composite shape descriptor
    tc_entry = next((e for e in data if e["name"] == "torque_curve"), None)
    check("torque_curve entry found", tc_entry is not None)
    if tc_entry:
        check("torque_curve has shape", "shape" in tc_entry)

    # mf52 entry should have a composite shape descriptor
    mf_entry = next((e for e in data if e["name"] == "mf52"), None)
    check("mf52 entry found", mf_entry is not None)
    if mf_entry:
        check("mf52 has shape", "shape" in mf_entry)

    # trajectory entry should have a composite shape descriptor
    traj_entry = next((e for e in data if e["name"] == "trajectory"), None)
    check("trajectory entry found", traj_entry is not None)
    if traj_entry:
        check("trajectory has shape", "shape" in traj_entry)


def test_404():
    print("\n── GET /nonexistent ───────────────────────────────────────────────")
    try:
        _get("/nonexistent")
        check("expected 404", False, "no exception raised")
    except urllib.error.HTTPError as e:
        check_eq("status 404", e.code, 404)


def test_lapsim_bad_json():
    print("\n── POST /lapsim (bad JSON) ─────────────────────────────────────────")
    req = urllib.request.Request(
        BASE_URL + "/lapsim", data=b"not json",
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=30):
            check("expected 400", False, "no exception raised")
    except urllib.error.HTTPError as e:
        check_eq("status 400", e.code, 400)
        body = json.loads(e.read().decode())
        check("error key present", "error" in body)


def test_lapsim_missing_field():
    print("\n── POST /lapsim (missing required field) ───────────────────────────")
    # Send a body that is valid JSON but omits required composite fields
    status, data = _post("/lapsim", {"total_weight": 595.0})
    check_eq("status 500 (missing fields)", status, 500)
    check("error key present", "error" in data)


def test_lapsim_defaults():
    print("\n── POST /lapsim (Panda defaults) ──────────────────────────────────")
    status, data = _post("/lapsim", _panda_body())
    check_eq("status 200", status, 200)
    check_keys("response keys", data,
               "laptime_1lap_s", "laptime_10lap_s", "score", "v_max_ms")
    check_in_range("laptime_1lap_s sane (40–200 s)", data["laptime_1lap_s"], 40.0, 200.0)
    check_in_range("score sane (0–300)",              data["score"],         0.0,  300.0)
    check_in_range("v_max_ms sane (10–100 m/s)",      data["v_max_ms"],     10.0, 100.0)
    return data


def test_lapsim_param_sensitivity(baseline: dict):
    print("\n── POST /lapsim (param sensitivity) ───────────────────────────────")
    # Heavier car should produce a slower lap time
    status, heavier = _post("/lapsim", _panda_body(total_weight=700.0))
    check_eq("status 200 (heavy car)", status, 200)
    check(
        "heavier car => slower",
        heavier["laptime_1lap_s"] > baseline["laptime_1lap_s"],
        f"heavy={heavier['laptime_1lap_s']} baseline={baseline['laptime_1lap_s']}",
    )

    # More downforce should change lap time
    status, aero = _post("/lapsim", _panda_body(Cl=4.0, Cd=1.6))
    check_eq("status 200 (aero change)", status, 200)
    check(
        "higher Cl => laptime changes",
        aero["laptime_1lap_s"] != baseline["laptime_1lap_s"],
    )


def test_lapsim_weak_torque_curve(baseline: dict):
    print("\n── POST /lapsim (weak flat torque curve, no throttle) ──────────────")
    weak_tc = {
        "rpm":      [0.0, 3000.0, 6000.0, 9000.0, 12500.0],
        "torque":   [70.0, 70.0,  70.0,   70.0,   70.0],
        "throttle": None,
    }
    status, data = _post("/lapsim", _panda_body(torque_curve=weak_tc))
    check_eq("status 200", status, 200)
    check_keys("response keys", data, "laptime_1lap_s", "score")
    check(
        "weak engine => slower",
        data["laptime_1lap_s"] > baseline["laptime_1lap_s"],
        f"weak={data['laptime_1lap_s']} baseline={baseline['laptime_1lap_s']}",
    )


def test_lapsim_torque_curve_with_throttle(baseline: dict):
    print("\n── POST /lapsim (torque curve with throttle column) ────────────────")
    # Two-throttle map: 50% rows at 40 N·m, 100% rows at 120 N·m.
    # Server should isolate the 100% rows for full-throttle interpolation.
    tc_with_throttle = {
        "rpm":      [0.0, 6000.0, 12500.0,  0.0, 6000.0, 12500.0],
        "torque":   [40.0, 40.0,   40.0,  120.0, 120.0,  120.0],
        "throttle": [0.5,  0.5,    0.5,    1.0,   1.0,    1.0],
    }
    status, data = _post("/lapsim", _panda_body(torque_curve=tc_with_throttle))
    check_eq("status 200", status, 200)
    check_keys("response keys", data, "laptime_1lap_s", "score")
    # 120 N·m full-throttle is close to stock — lap time should be reasonable
    check_in_range(
        "laptime with throttle map sane (40–200 s)",
        data["laptime_1lap_s"], 40.0, 200.0,
    )


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    print(f"Testing server at {BASE_URL}")

    test_root()
    test_health()
    test_lapsim_args()
    test_404()
    test_lapsim_bad_json()
    test_lapsim_missing_field()

    baseline = test_lapsim_defaults()
    if baseline:
        test_lapsim_param_sensitivity(baseline)
        test_lapsim_weak_torque_curve(baseline)
        test_lapsim_torque_curve_with_throttle(baseline)

    print()
    if _failures:
        print(f"\033[31m{len(_failures)} test(s) FAILED:\033[0m")
        for f in _failures:
            print(f"  • {f}")
        sys.exit(1)
    else:
        print("\033[32mAll tests PASSED.\033[0m")


if __name__ == "__main__":
    main()
