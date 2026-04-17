# precompile_execution.jl
# Exercises all major simulation code paths so PackageCompiler can trace them.
# Uses synthetic data to avoid requiring fixture files at compile time.

include("../VogelSIM.jl")
using .VogelSIM

# ── Synthetic tire params (MF52 expects 2 vectors + Fz0=800N) ─────────────────
# Fx: p[1..15] used — PCX1, PDX1, PDX2, PDX3, PEX1-4, PKX1-3, PHX1-2, PVX1-2
_FX = [1.65, 1.20, -0.08, 0.0,  # p[1..4]  PCX1 PDX1 PDX2 PDX3
       0.0,  0.0,  0.0,  0.0,   # p[5..8]  PEX1-4
       15.0, 0.0,  0.0,         # p[9..11] PKX1-3
       0.0,  0.0,               # p[12..13] PHX1-2
       0.0,  0.0]               # p[14..15] PVX1-2

# Fy: p[1..27] — PCY1, PDY1-3, PEY1-5, PKY1-7, PHY1-2, PVY1-4, PPY1-5
_FY = [1.30, 1.15, -0.10, 0.0,  # p[1..4]   PCY1 PDY1 PDY2 PDY3
       0.0,  0.0,  0.0,  0.0,   # p[5..8]   PEY1-4
       0.0,                     # p[9]      PEY5
       12.0, 1.5,  0.0,         # p[10..12] PKY1-3
       0.0,  0.0,  0.0,  0.0,   # p[13..16] PKY4-7
       0.0,  0.0,               # p[17..18] PHY1-2
       0.0,  0.0,  0.0,  0.0,   # p[19..22] PVY1-4
       0.0,  0.0,  0.0,  0.0,  0.0]  # p[23..27] PPY1-5

mf52 = MF52(_FX, _FY)

# ── Synthetic torque curve ─────────────────────────────────────────────────────
tc = TorqueCurve(
    [0.0, 3000.0, 6000.0, 9000.0, 12500.0],
    [95.0, 105.0, 110.0, 100.0,  80.0],
    nothing,
)

# ── Params (all SI, no file I/O) ──────────────────────────────────────────────
params = Params(
    595.0 * 4.4482216153,   # total_weight  (already N)
    0.46,                    # weight_dist_f
    12.0866 / 39.37,         # cg_height  (m)
    1.535,                   # wheelbase
    48.875 * 0.0254,         # trackwidth_f (m)
    48.875 * 0.0254,         # trackwidth_r (m)
    0.028, 0.036,            # rollc_f/r
    0.0, 0.0,                # asquat, adive
    1184.0, 1184.0,          # torsional_rigidity, tr_nom
    31430.0, 38375.0,        # ride_rate_f/r
    486.0 * 57.2958,         # k_phi_f (rad-scaled)
    650.0 * 57.2958,         # k_phi_r
    0.2032,                  # tire_radius
    0.0, 0.0,                # static_camber_f/r
    0.0, 0.0,                # camber_gain_f/r
    0.6, 0.6,                # friction_scaling_x/y
    2.8, 1.1, 0.51,          # Cl, Cd, CoP
    tc,
    38.0/18.0,               # primary_reduction
    [33/12, 32/16, 30/18, 26/18, 30/23, 29/24],
    6,                       # num_gears
    37.0/11.0,               # final_drive
    12500.0,                 # shiftpoint
    0.85,                    # drivetrain_losses
    0.25,                    # shift_time
    false,                   # diff_locked
    0.5,                     # LLTD
    # Derived
    595.0 * 4.4482216153 * 0.46,
    595.0 * 4.4482216153 * 0.54,
    48.875 * 0.0254,
)

# ── Synthetic oval trajectory (50 points) ─────────────────────────────────────
let
    n   = 50
    θ   = range(0, 2π; length=n+1)[1:n]
    x_m = 30.0 .* cos.(θ)
    y_m = 15.0 .* sin.(θ)
    r   = fill(15.0, n)
    κ   = fill(1.0/15.0, n)

    traj = Trajectory(x_m, y_m, r, κ, 3.5, 36.0)

    # ── GGV ───────────────────────────────────────────────────────────────────
    gear_tot = params.gear_ratios[end] * params.final_drive * params.primary_reduction
    v_max    = params.shiftpoint / (gear_tot / params.tire_radius * 60.0 / (2π))

    ggv = GGV(params, mf52, gear_tot, v_max; calc_lateral=true)
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
    v = Vehicle(params, mf52, traj; calc_lateral=true, mesh_resolution=10)
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
    max_throttle_torque(tc, 8000.0)
end
