# precompile_execution.jl
# Exercises all major simulation code paths so PackageCompiler can trace them.
# Uses real data files copied into the Docker build context at /app/data/.

include("../VogelSIM.jl")
using .VogelSIM

const DATA_DIR = joinpath(@__DIR__, "..", "..", "data")

# ── Load real MF52 params from .mat files ─────────────────────────────────────
mf52 = MF52(DATA_DIR)

# ── Load real torque curve ────────────────────────────────────────────────────
params = Panda(DATA_DIR)

# ── Load real trajectory ──────────────────────────────────────────────────────
traj_file = joinpath(DATA_DIR, "trajectory", "23_michigan_endurance_ft.csv")
traj = Trajectory(traj_file, true, 3.5, 36.0)

# ── GGV ───────────────────────────────────────────────────────────────────────
let
    gear_tot = params.gear_ratios[end] * params.final_drive * params.primary_reduction
    v_max    = params.shiftpoint / (gear_tot / params.tire_radius * 60.0 / (2π))

    # Use calc_lateral=false to skip the expensive NLsolve path during precompile
    ggv = GGV(params, mf52, gear_tot, v_max; calc_lateral=false)
    generate!(ggv)

    # Exercise GGV query functions post-fit
    for v in [5.0, 15.0, 25.0]
        calc_grip_lim_max_accel(ggv, v)
        calc_power_lim_max_accel(ggv, max(7.5, v))
        calc_decel(ggv, v)
        evaluate(ggv.accel_capability,    v)
        evaluate(ggv.braking_capability,  v)
        evaluate(ggv.lateral_capability,  v)
    end
    for r in [8.0, 20.0]
        evaluate(ggv.cornering_capability, r)
    end

    # ── Full lap simulation ───────────────────────────────────────────────────
    v = Vehicle(params, mf52, traj; calc_lateral=false, mesh_resolution=10)
    simulate_endurance!(v)

    # ── VehicleState post-processing ─────────────────────────────────────────
    si = StateInput(Ax=0.3, Ay=0.5, v=15.0, r=15.0)
    vs = VehicleState(mf52, params)
    eval!(vs, si)

    # ── TireState ────────────────────────────────────────────────────────────
    ts = TireState(mf52, params.friction_scaling_x, params.friction_scaling_y)
    ts.Fz = 1200.0
    eval_Fx!(ts)
    ts.alpha = 3.0
    eval_Fy!(ts)

    # ── MF52 direct calls ─────────────────────────────────────────────────────
    mf52_Fx(mf52, 1000.0, 0.1, 0.0)
    mf52_Fy(mf52, 1000.0, 5.0, 0.0)

    # ── Fitting ───────────────────────────────────────────────────────────────
    xs = Float64.(1:10); ys = xs .^ 2
    pf = polyfit(xs, ys, 3)
    evaluate(pf, 5.0)
    sp = csaps(xs, ys)
    evaluate(sp, 5.0)

    # ── max_throttle_torque ───────────────────────────────────────────────────
    max_throttle_torque(params.torque_curve, 8000.0)
end
