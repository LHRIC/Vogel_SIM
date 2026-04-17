# TrackPath.jl — included into module VogelSIM
#
# Provides a spline-based track representation that computes arc-length
# parameterization and curvature analytically, replacing the pre-computed
# CSV trackmap workflow.

using Dierckx

# ── SplineCurve ────────────────────────────────────────────────────────────────
# Wraps a pair of pre-fitted Dierckx.Spline1D objects that together define a
# parametric curve (x(t), y(t)) over a given parameter domain.
#
# The parameter `t` must match the knot/coefficient convention of the splines
# you supply — typically [0, 1] normalized or physical arc length. Be
# consistent: whichever convention is used here must match what produced the
# coefficient arrays.

struct SplineCurve
    x_spline::Spline1D
    y_spline::Spline1D
    t_range::Tuple{Float64,Float64}
end

"""
    SplineCurve(t_knots, x_vals, y_vals; bc="nearest", s=0.0)

Fit a cubic B-spline through the supplied (t, x) and (t, y) control points.
`s` is the Dierckx smoothing factor (0 = interpolating, >0 = smoothing).
"""
function SplineCurve(t_knots::AbstractVector{<:Real},
                     x_vals::AbstractVector{<:Real},
                     y_vals::AbstractVector{<:Real};
                     bc::String="nearest",
                     s::Float64=0.0)
    t = Float64.(t_knots)
    x_sp = Spline1D(t, Float64.(x_vals); bc=bc, s=s)
    y_sp = Spline1D(t, Float64.(y_vals); bc=bc, s=s)
    return SplineCurve(x_sp, y_sp, (t[1], t[end]))
end

"""
    evaluate(sc::SplineCurve, t) -> (x, y)

Evaluate the parametric curve at parameter value(s) `t`.
"""
function evaluate_curve(sc::SplineCurve, t::Real)
    return (sc.x_spline(t), sc.y_spline(t))
end

# ── Gate ───────────────────────────────────────────────────────────────────────

struct Gate
    point::Tuple{Float64,Float64}   # gate midpoint (x, y) in metres
    normal::Tuple{Float64,Float64}  # unit normal to the gate line
end

"""
    make_gate(point, normal) -> Gate

Construct a `Gate`, auto-normalizing the supplied normal vector.
`point` and `normal` may be any 2-element indexable collection (Tuple, Vector, etc.).

Using `make_gate` rather than the bare `Gate` constructor ensures normalization
always occurs — the raw struct constructor is kept for internal use only.
"""
function make_gate(point, normal)
    nx, ny = Float64(normal[1]), Float64(normal[2])
    len = sqrt(nx^2 + ny^2)
    len < 1e-12 && error("Gate normal vector has near-zero magnitude")
    return Gate(
        (Float64(point[1]), Float64(point[2])),
        (nx/len, ny/len)
    )
end

# ── Path ───────────────────────────────────────────────────────────────────────
# Arc-length parameterized raceline derived from a SplineCurve.

struct Path
    s::Vector{Float64}       # arc-length samples [0 … L] (metres)
    x::Vector{Float64}       # x(s)
    y::Vector{Float64}       # y(s)
    dx_ds::Vector{Float64}   # dx/ds
    dy_ds::Vector{Float64}   # dy/ds
    d2x_ds2::Vector{Float64} # d²x/ds²
    d2y_ds2::Vector{Float64} # d²y/ds²
    kappa::Vector{Float64}   # signed curvature κ
    radii::Vector{Float64}   # |1/κ|, clamped to [r_min, r_max]
    length::Float64          # total arc length L (metres)
end

"""
    build_path(curve, n_samples, r_min, r_max; smooth) -> Path

Build an arc-length parameterized `Path` from a `SplineCurve`.

Steps:
1. Sample the native-parameter spline uniformly → (x, y) points.
2. Integrate chord lengths → arc-length vector `s`.
3. Fit *smoothing* cubic splines x(s) and y(s) (smoothing suppresses
   oscillatory second derivatives from numerical arc-length nodes).
4. Evaluate analytic first and second derivatives from Dierckx.
5. Compute signed curvature κ = (x'y'' − y'x'') / (x'² + y'²)^(3/2).
6. Clamp |1/κ| to [r_min, r_max].

`smooth`: Dierckx smoothing factor for the arc-length splines (default 0.1).
A small positive value is strongly recommended; use 0.0 only for very clean
synthetic inputs (e.g. unit tests with a perfect circle).
"""
function build_path(curve::SplineCurve,
                    n_samples::Int,
                    r_min::Float64,
                    r_max::Float64;
                    smooth::Float64=0.1)::Path

    n_samples >= 10 || error("n_samples must be ≥ 10")

    # 1. Sample uniformly in native parameter t
    t0, t1 = curve.t_range
    t_vals = range(t0, t1, length=n_samples)
    xs = curve.x_spline.(t_vals)
    ys = curve.y_spline.(t_vals)

    # 2. Cumulative arc-length (trapezoidal — exact for piecewise linear)
    s = zeros(n_samples)
    for i in 2:n_samples
        dx = xs[i] - xs[i-1]
        dy = ys[i] - ys[i-1]
        s[i] = s[i-1] + sqrt(dx^2 + dy^2)
    end
    L = s[end]
    L > 0.0 || error("Degenerate SplineCurve: total arc length is zero")

    # 3. Fit smoothing splines x(s) and y(s) over the arc-length grid.
    #    The smoothing factor `smooth` is scaled by the number of points so
    #    the default behaves consistently regardless of n_samples.
    sf = smooth * n_samples
    xs_sp = Spline1D(s, xs; bc="nearest", s=sf)
    ys_sp = Spline1D(s, ys; bc="nearest", s=sf)

    # 4. Evaluate splines and their analytic derivatives at the same s nodes
    x_out   = xs_sp.(s)
    y_out   = ys_sp.(s)
    dx_ds   = Dierckx.derivative(xs_sp, s; nu=1)
    dy_ds   = Dierckx.derivative(ys_sp, s; nu=1)
    d2x_ds2 = Dierckx.derivative(xs_sp, s; nu=2)
    d2y_ds2 = Dierckx.derivative(ys_sp, s; nu=2)

    # 5. Signed curvature κ = (x'y'' − y'x'') / (x'² + y'²)^(3/2)
    kappa = (dx_ds .* d2y_ds2 .- dy_ds .* d2x_ds2) ./
            (dx_ds.^2 .+ dy_ds.^2).^1.5

    # 6. Radii: |1/κ|, guard against κ ≈ 0 (straight sections)
    radii = clamp.(abs.(1.0 ./ replace_zeros(kappa)), r_min, r_max)

    return Path(s, x_out, y_out, dx_ds, dy_ds, d2x_ds2, d2y_ds2,
                kappa, radii, L)
end

# Replace near-zero curvature with a tiny value to avoid Inf radii before clamping.
function replace_zeros(v::Vector{Float64}, tol::Float64=1e-10)
    out = copy(v)
    for i in eachindex(out)
        abs(out[i]) < tol && (out[i] = tol)
    end
    return out
end

# ── TrackPath ──────────────────────────────────────────────────────────────────

struct TrackPath
    inner_bound::SplineCurve   # inner track boundary spline
    outer_bound::SplineCurve   # outer track boundary spline
    raceline::Path             # arc-length parameterized raceline
    start_gate::Gate           # gate at s = 0
    end_gate::Gate             # gate at s = L
    n_loops::Int               # number of laps (stored; not yet wired into lap sim)
end

"""
    TrackPath(inner, outer, raceline_spline, start_gate, end_gate, n_loops;
              n_samples=500, r_min=3.5, r_max=36.0, smooth=0.1)

Construct a `TrackPath` from two boundary `SplineCurve`s and a raceline
`SplineCurve`. The raceline is immediately reparameterized by arc length and
curvature is computed analytically.

# Arguments
- `inner`, `outer`: `SplineCurve` objects for the track boundaries (stored for
  future use; not consumed by the lap sim directly).
- `raceline_spline`: `SplineCurve` for the driving line.
- `start_gate`, `end_gate`: `Gate` structs marking the lap start/finish.
- `n_loops`: number of laps to simulate (stored; lap sim currently runs one lap).
- `n_samples`: number of arc-length sample points (≥ 500 recommended).
- `r_min`, `r_max`: curvature radius clamp limits in metres.
- `smooth`: Dierckx smoothing factor for the arc-length reparameterization.
"""
function TrackPath(inner::SplineCurve,
                   outer::SplineCurve,
                   raceline_spline::SplineCurve,
                   start_gate::Gate,
                   end_gate::Gate,
                   n_loops::Int;
                   n_samples::Int=500,
                   r_min::Float64=3.5,
                   r_max::Float64=36.0,
                   smooth::Float64=0.1)

    n_loops >= 1 || error("n_loops must be ≥ 1")
    path = build_path(raceline_spline, n_samples, r_min, r_max; smooth=smooth)
    return TrackPath(inner, outer, path, start_gate, end_gate, n_loops)
end

# ── Adapter ────────────────────────────────────────────────────────────────────

"""
    to_trajectory(tp::TrackPath) -> Trajectory

Convert a `TrackPath` to the discrete `Trajectory` struct expected by the
existing lap simulation code. The `TrackPath` object itself should be retained
on the `Vehicle` for future use (track-width queries, path optimization, etc.).
"""
function to_trajectory(tp::TrackPath)::Trajectory
    p = tp.raceline
    return Trajectory(
        (copy(p.x), copy(p.y)),
        copy(p.radii),
        copy(p.kappa),
        length(p.s)
    )
end
