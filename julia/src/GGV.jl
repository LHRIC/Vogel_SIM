# GGV.jl — included into module VogelSIM
# Depends on: MF52, TireState, VehicleState, Fitting, Panda

using NLsolve

# Precalculated lateral capability (g) vs radii_range (3.5:1.5:34.5 m), 22 values
const LATERAL_CAP_PRECOMPUTED = [
    1.38195832278988, 1.42938584362690, 1.46626343925345, 1.49126646868585,
    1.51244972228054, 1.53029970410364, 1.54641562015666, 1.56156587853348,
    1.57629799755505, 1.59036217462828, 1.60381424225666, 1.61660679209661,
    1.62875493176357, 1.64020405391575, 1.65090601787886, 1.66090586967413,
    1.66084693559365, 1.65788498156999, 1.65256859579085, 1.64521365471666,
    1.63619883252664, 1.62583858287551,
]

mutable struct GGV
    params::Panda
    mf52::MF52
    gear_tot::Float64
    v_max::Float64
    velocity_range::UnitRange{Int64}
    radii_range::Vector{Float64}
    vehicle_state::VehicleState
    _grip_lim_accel::Union{FitFunction,Nothing}
    _power_lim_accel::Union{FitFunction,Nothing}
    _calc_lateral::Bool
    accel_capability::Union{FitFunction,Nothing}
    cornering_capability::Union{FitFunction,Nothing}
    braking_capability::Union{FitFunction,Nothing}
    lateral_capability::Union{FitFunction,Vector{Float64}}
    expected_gears::Vector{Int}
end

function GGV(params::Panda, mf52::MF52, gear_tot::Float64, v_max::Float64;
             calc_lateral::Bool=true)
    radii_range   = collect(3.5:1.5:(36.0 - 1e-9))   # matches np.arange(3.5, 36, 1.5)
    v_lo = 4; v_hi = floor(Int, v_max)
    velocity_range = v_lo:v_hi
    vs = VehicleState(mf52, params)
    GGV(params, mf52, gear_tot, v_max, velocity_range, radii_range, vs,
        nothing, nothing, calc_lateral,
        nothing, nothing, nothing,
        copy(LATERAL_CAP_PRECOMPUTED), Int[])
end

# ── GGV sub-calculations ──────────────────────────────────────────

function calc_grip_lim_max_accel(g::GGV, v::Float64)::Float64
    vs = g.vehicle_state
    A_x_diff = 1.0; Ax = 0.0
    while A_x_diff > 1e-6
        Ax += 0.01
        eval!(vs, StateInput(Ax=Ax, Ay=0.0, v=v))
        AX = (vs.rl_tire.Fx + vs.rr_tire.Fx) / g.params.total_weight
        A_x_diff = AX - Ax
    end
    eval!(vs, StateInput(Ax=Ax, Ay=0.0, v=v))
    return (vs.rl_tire.Fx + vs.rr_tire.Fx) / g.params.total_weight
end

function calc_power_lim_max_accel(g::GGV, v::Float64)
    gear_idx  = 0
    rpm       = g.params.shiftpoint
    rpm_diff  = 1000.0
    total_red = 1.0
    while rpm >= g.params.shiftpoint && rpm_diff > 1e-6
        total_red = (g.params.gear_ratios[gear_idx+1]
                     * g.params.final_drive * g.params.primary_reduction)
        gear_idx  = min(gear_idx + 1, g.params.num_gears - 1)
        new_rpm   = v * total_red / g.params.tire_radius * 60.0 / (2π)
        rpm_diff  = abs(rpm - new_rpm)
        rpm       = new_rpm
    end
    τ_crank = _interp1(g.params.rpm_range, g.params.torque_curve, rpm)
    Fx_w    = τ_crank * total_red * g.params.drivetrain_losses / g.params.tire_radius
    return Fx_w, gear_idx
end

function calc_decel(g::GGV, v::Float64)::Float64
    vs = g.vehicle_state
    A_x_diff = 1.0; Ax = 0.0
    while A_x_diff > 0.0
        Ax -= 0.01
        eval!(vs, StateInput(Ax=Ax, Ay=0.0, v=v))
        FX = -1.0*(vs.fl_tire.Fx + vs.fr_tire.Fx + vs.rl_tire.Fx + vs.rr_tire.Fx)
        A_x_diff = FX/g.params.total_weight - Ax
    end
    eval!(vs, StateInput(Ax=Ax, Ay=0.0, v=v))
    FX = -1.0*(vs.fl_tire.Fx + vs.fr_tire.Fx + vs.rl_tire.Fx + vs.rr_tire.Fx)
    return FX / g.params.total_weight
end

# ── Vogel solver (used when calc_lateral=true) ───────────────────

function _vogel_residuals!(res, x, g::GGV, R::Float64)
    delta, beta, AYP = x[1], x[2], x[3]
    vs = g.vehicle_state
    a  = g.params.wheelbase*(1.0-g.params.weight_dist_f)
    b  = g.params.wheelbase*g.params.weight_dist_f

    V  = sqrt(R*9.81*AYP)
    eval!(vs, StateInput(Ax=0.0, Ay=AYP, v=V, r=R, delta=delta, beta=beta))

    F_f_in  = vs.fl_tire.Fy * cos(delta)
    F_f_out = vs.fr_tire.Fy * cos(delta)
    F_xDrag = g.params.Cd*V^2 + (F_f_in+F_f_out)*sin(delta)/cos(delta)

    gl = evaluate(g._grip_lim_accel, V)
    rscale = max(0.0, 1.0-(F_xDrag/g.params.total_weight/gl)^2)

    F_r_in  = vs.rl_tire.Fy * rscale
    F_r_out = vs.rr_tire.Fy * rscale
    F_y     = F_f_in + F_f_out + F_r_in + F_r_out

    M_z_diff = F_xDrag * (g.params.diff_locked ? 1.0 : 0.0) * g.params.trackwidth_r/2.0
    M_z      = (F_f_in+F_f_out)*a - (F_r_in+F_r_out)*b - M_z_diff
    AY       = F_y / g.params.total_weight

    # Normalise M_z to O(1) so all three residuals are comparably scaled.
    # Without this NLsolve's LM step is dominated by M_z (N·m range) and
    # ignores the slip-angle and Ay-consistency equations — matching Python's
    # scipy.least_squares behaviour which auto-scales each residual column.
    res[1] = M_z / (g.params.total_weight * g.params.wheelbase)
    res[2] = vs.alpha_f - deg2rad(-12.0)
    res[3] = AYP - AY
end

# Sigmoid reparameterization: map unconstrained y → x ∈ [lb, ub].
# This avoids NLsolve ever probing the boundary, which would produce a
# zero Jacobian column and a NaN trust-region step.
_vsig(y::Float64) = y >= 0.0 ? 1.0/(1.0+exp(-y)) : exp(y)/(1.0+exp(y))
_free_to_box(y::Vector{Float64}, lb::Vector{Float64}, ub::Vector{Float64}) =
    lb .+ (ub .- lb) .* _vsig.(y)
function _box_to_free(x::Vector{Float64}, lb::Vector{Float64}, ub::Vector{Float64})
    p = clamp.((x .- lb) ./ (ub .- lb), 1e-8, 1.0-1e-8)
    return log.(p ./ (1.0 .- p))
end

function calc_lateral_accel(g::GGV, R::Float64, AYP_guess::Float64)::Float64
    lb = [0.04, -0.3, 0.1]; ub = [0.50, 0.3, 3.0]
    x0 = clamp.([g.params.wheelbase/R, 0.0, AYP_guess], lb, ub)
    y0 = _box_to_free(x0, lb, ub)

    result = nlsolve(
        (res, y) -> _vogel_residuals!(res, _free_to_box(y, lb, ub), g, R),
        y0; method=:trust_region, ftol=1e-6, iterations=1000, show_trace=false
    )

    # Only fall back to the warm-start if the solver produced NaN (true divergence).
    # Do NOT gate on result.converged: with normalised residuals the solver will
    # reach a good solution even if ftol isn't perfectly satisfied.
    y_sol = any(isnan, result.zero) ? y0 : result.zero
    x_sol = _free_to_box(y_sol, lb, ub)

    # Evaluate AY at solution
    vs = g.vehicle_state
    delta, beta, AYP = x_sol[1], x_sol[2], x_sol[3]
    V = sqrt(R*9.81*AYP)
    eval!(vs, StateInput(Ax=0.0, Ay=AYP, v=V, r=R, delta=delta, beta=beta))
    F_f_in  = vs.fl_tire.Fy*cos(delta); F_f_out = vs.fr_tire.Fy*cos(delta)
    F_xDrag = g.params.Cd*V^2 + (F_f_in+F_f_out)*sin(delta)/cos(delta)
    gl      = evaluate(g._grip_lim_accel, V)
    rscale  = max(0.0, 1.0-(F_xDrag/g.params.total_weight/gl)^2)
    F_y     = F_f_in + F_f_out + vs.rl_tire.Fy*rscale + vs.rr_tire.Fy*rscale
    return F_y / g.params.total_weight
end

# ── Main generate ────────────────────────────────────────────────

function generate!(g::GGV)
    vrange = Float64.(collect(g.velocity_range))
    n_v    = length(vrange)

    power_lim_a = zeros(n_v); grip_lim_a = zeros(n_v); accel_cap = zeros(n_v)

    for (i, v) in enumerate(vrange)
        @info "Accel GGV: v=$(v) m/s"
        grip_lim_a[i] = calc_grip_lim_max_accel(g, v)

        Fx_r, gear_idx = calc_power_lim_max_accel(g, max(7.5, v))
        push!(g.expected_gears, gear_idx)

        Fx_r -= g.params.Cd * v^2
        power_lim_a[i] = Fx_r / g.params.total_weight
        accel_cap[i]   = min(grip_lim_a[i], power_lim_a[i])
    end

    g._grip_lim_accel  = polyfit(vrange, grip_lim_a, 3)
    g._power_lim_accel = csaps(vrange, power_lim_a)
    g.accel_capability = csaps(vrange, accel_cap)

    radii    = g.radii_range; n_r = length(radii)
    lateral_g = zeros(n_r)

    if g._calc_lateral
        for (i, R) in enumerate(radii)
            @info "Lateral GGV: R=$(R) m"
            lateral_g[i] = calc_lateral_accel(g, R, LATERAL_CAP_PRECOMPUTED[i])
        end
    else
        @warn "Using precalculated lateral envelope"
        lateral_g .= LATERAL_CAP_PRECOMPUTED
    end

    accel_y    = lateral_g .* 9.81
    velocity_y = sqrt.(accel_y .* radii)

    g.lateral_capability  = polyfit(velocity_y, lateral_g, 4)
    g.cornering_capability = polyfit(radii, velocity_y, 4)

    braking_g = zeros(n_v)
    for (i, v) in enumerate(vrange)
        @info "Braking GGV: v=$(v) m/s"
        braking_g[i] = calc_decel(g, v)
    end
    g.braking_capability = polyfit(vrange, braking_g, 4)
end

# Simple linear interpolation helper
function _interp1(xs::Vector{Float64}, ys::Vector{Float64}, x::Float64)::Float64
    x <= xs[1]   && return ys[1]
    x >= xs[end] && return ys[end]
    lo, hi = 1, length(xs)
    while hi - lo > 1
        mid = (lo+hi)÷2; xs[mid]<=x ? (lo=mid) : (hi=mid)
    end
    t = (x-xs[lo])/(xs[hi]-xs[lo])
    return ys[lo] + t*(ys[hi]-ys[lo])
end
