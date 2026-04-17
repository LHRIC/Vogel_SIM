# debug_lapsim.jl
# Run from julia/ directory:  julia --project=. test/debug_lapsim.jl
#
# Prints key intermediate values at every stage of the simulation so that
# the Julia and Python results can be compared line-by-line.

using Printf

include(joinpath(@__DIR__, "..", "src", "VogelSIM.jl"))
using .VogelSIM

BASE = joinpath(@__DIR__, "..", "..")

# ── Build vehicle ─────────────────────────────────────────────────────────────
params = Panda(joinpath(BASE, "python"))
mf52   = MF52(joinpath(BASE, "python"))
traj   = Trajectory(
    joinpath(BASE, "python", "trajectory", "23_michigan_endurance_ft.csv"),
    true, 3.5, 36.0
)

println("=== PARAMS ===")
@printf("  total_weight    = %.4f N\n",  params.total_weight)
@printf("  weight_dist_f   = %.4f\n",    params.weight_dist_f)
@printf("  cg_height       = %.6f m\n",  params.cg_height)
@printf("  trackwidth_f    = %.6f m\n",  params.trackwidth_f)
@printf("  trackwidth_r    = %.6f m\n",  params.trackwidth_r)
@printf("  tire_radius     = %.6f m\n",  params.tire_radius)
@printf("  Cl=%.3f  Cd=%.3f  CoP=%.3f\n", params.Cl, params.Cd, params.CoP)
@printf("  k_phi_f=%.4f  k_phi_r=%.4f\n", params.k_phi_f, params.k_phi_r)
@printf("  ride_rate_f=%.1f  ride_rate_r=%.1f N/mm\n", params.ride_rate_f, params.ride_rate_r)
@printf("  friction_x=%.3f  friction_y=%.3f\n", params.friction_scaling_x, params.friction_scaling_y)

println("\n=== TRAJECTORY ===")
@printf("  num_points = %d\n",  traj.num_points)
@printf("  radii[1:5] = %s\n",  string(traj.radii[1:5]))
@printf("  radii min/max = %.4f / %.4f m\n", minimum(traj.radii), maximum(traj.radii))

# ── GGV generation ────────────────────────────────────────────────────────────
gear_tot = params.gear_ratios[end] * params.final_drive * params.primary_reduction
v_max    = params.shiftpoint / (gear_tot / params.tire_radius * 60.0 / (2π))
@printf("\n=== GGV SETUP ===\n")
@printf("  gear_tot = %.6f\n", gear_tot)
@printf("  v_max    = %.4f m/s\n", v_max)

ggv = GGV(params, mf52, gear_tot, v_max; calc_lateral=true)

println("\n--- GGV raw accel values at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]
    grip  = calc_grip_lim_max_accel(ggv, v)
    Fx_w, gear_idx = calc_power_lim_max_accel(ggv, max(7.5, v))
    power = (Fx_w - params.Cd * v^2) / params.total_weight
    accel = min(grip, power)
    @printf("  v=%5.1f m/s | grip=%.4f g | power=%.4f g | accel=%.4f g | gear=%d\n",
            v, grip, power, accel, gear_idx)
end

println("\n--- GGV raw braking values at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]
    decel = calc_decel(ggv, v)
    @printf("  v=%5.1f m/s | decel=%.4f g\n", v, decel)
end

println("\n--- Lateral envelope (precomputed) ---")
for (i, R) in enumerate(ggv.radii_range)
    lat_g = ggv.lateral_capability[i]   # still a Vector before generate!
    v_corn = sqrt(lat_g * 9.81 * R)
    @printf("  R=%5.2f m | lat=%.4f g | v_corner=%.4f m/s\n", R, lat_g, v_corn)
end

# Run full generate!
println("\n=== Running generate!() ===")
generate!(ggv)

println("\n--- Fitted accel_capability at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]
    @printf("  accel_capability(%.1f) = %.6f g\n", v, evaluate(ggv.accel_capability, v))
end

println("\n--- Fitted braking_capability at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]
    @printf("  braking_capability(%.1f) = %.6f g\n", v, evaluate(ggv.braking_capability, v))
end

println("\n--- Fitted cornering_capability at selected radii ---")
for R in [5.0, 8.0, 12.0, 20.0, 30.0]
    @printf("  cornering_capability(%.1f m) = %.4f m/s\n", R, evaluate(ggv.cornering_capability, R))
end

println("\n--- Fitted lateral_capability at selected velocities ---")
for v in [5.0, 10.0, 15.0, 20.0, 25.0]
    @printf("  lateral_capability(%.1f m/s) = %.6f g\n", v, evaluate(ggv.lateral_capability, v))
end

# ── Forward pass debug ────────────────────────────────────────────────────────
v = Vehicle(params, mf52, traj; calc_lateral=true, mesh_resolution=10)
# Inject the already-generated ggv so we don't recompute
v.ggv.accel_capability    = ggv.accel_capability
v.ggv.braking_capability  = ggv.braking_capability
v.ggv.cornering_capability = ggv.cornering_capability
v.ggv.lateral_capability  = ggv.lateral_capability
v.ggv.expected_gears       = ggv.expected_gears
v.ggv._grip_lim_accel      = ggv._grip_lim_accel

println("\n=== FORWARD PASS (first 5 trajectory segments) ===")

# Inline solve_dt for debug script
function _solve_dt(delta_d, vel, ax)
    a_c = 0.5*9.81*ax; b_c = vel; c_c = -delta_d
    abs(a_c) < 1e-12 && return delta_d/vel
    disc = b_c^2 - 4.0*a_c*c_c
    disc < 0.0 && return delta_d/vel
    sq = sqrt(disc)
    t1 = (-b_c+sq)/(2.0*a_c); t2 = (-b_c-sq)/(2.0*a_c)
    cands = filter(t->t>0.0, [t1,t2])
    isempty(cands) && return delta_d/vel
    return maximum(cands)
end

let vel = 20.0 * 0.3048
    for pi in 1:min(5, traj.num_points-1)
        x1 = traj.points[1][pi]; y1 = traj.points[2][pi]
        x2 = traj.points[1][pi+1]; y2 = traj.points[2][pi+1]
        seg = sqrt((x1-x2)^2 + (y1-y2)^2)
        r   = traj.radii[pi]
        v_max_seg = min(v_max, evaluate(ggv.cornering_capability, r))
        AX_cap = evaluate(ggv.accel_capability, vel)
        AY_cap = evaluate(ggv.lateral_capability, vel)
        AY_act = vel^2 / (r * 9.81)
        ax_pot = AX_cap * sqrt(1.0 - (min(AY_cap, AY_act)/AY_cap)^2)
        ddelta = seg / 10
        dt     = _solve_dt(ddelta, vel, ax_pot)
        dv     = 9.81 * ax_pot * dt
        @printf("seg %d | seg=%.4fm r=%.2fm | vel=%.4f→%.4f m/s | AX=%.4f AY_cap=%.4f AY_act=%.4f | ax_pot=%.4f | dt=%.6fs\n",
                pi, seg, r, vel, min(v_max_seg, vel+dv), AX_cap, AY_cap, AY_act, ax_pot, dt)
        vel = min(v_max_seg, vel + dv)
    end
end

# Full forward pass
simulate_forwards!(v, 20.0*0.3048)
println("\n--- Forward pass summary ---")
@printf("  max velocity_f = %.4f m/s\n", maximum(v.velocity_f))
@printf("  min velocity_f = %.4f m/s\n", minimum(v.velocity_f))
@printf("  total fwd time = %.4f s\n",   maximum(v.time))
@printf("  velocity_f[1:5]  = %s m/s\n", string(round.(v.velocity_f[1:5]; digits=4)))
@printf("  velocity_f[end-4:end] = %s m/s\n", string(round.(v.velocity_f[end-4:end]; digits=4)))

# Full reverse pass
simulate_reverse!(v)
println("\n--- Reverse pass summary ---")
@printf("  max velocity_r = %.4f m/s\n", maximum(v.velocity_r))
@printf("  min velocity_r = %.4f m/s\n", minimum(v.velocity_r))
@printf("  velocity_r[1:5]  = %s m/s\n", string(round.(v.velocity_r[1:5]; digits=4)))
@printf("  velocity_r[end-4:end] = %s m/s\n", string(round.(v.velocity_r[end-4:end]; digits=4)))

# Second forward pass and merge
simulate_forwards!(v, v.velocity_r[1])

for i in eachindex(v.count)
    v.dist[i] = v.dist_f[i]
    if v.velocity_f[i] < v.velocity_r[i]
        v.velocity[i] = v.velocity_f[i]
    else
        v.velocity[i] = v.velocity_r[i]
    end
end

println("\n--- Merged velocity summary ---")
@printf("  max velocity   = %.4f m/s\n", maximum(v.velocity))
@printf("  min velocity   = %.4f m/s\n", minimum(v.velocity))
n_fwd = count(i -> v.velocity_f[i] <= v.velocity_r[i], 1:length(v.velocity))
n_rev = length(v.velocity) - n_fwd
@printf("  fwd-pass wins  = %d points (%.1f%%)\n", n_fwd, 100*n_fwd/length(v.velocity))
@printf("  rev-pass wins  = %d points (%.1f%%)\n", n_rev, 100*n_rev/length(v.velocity))

# Final laptime
laptime = simulate_endurance!(v)
println("\n=== RESULT ===")
@printf("  laptime = %.4f s\n", laptime)
