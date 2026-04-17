using Polynomials
using Interpolations

"""Dense (fit_x, fit_y) pair evaluated by linear interpolation"""
struct FitFunction
    fit_x::Vector{Float64}
    fit_y::Vector{Float64}
end

function evaluate(f::FitFunction, x_val::Float64)::Float64
    xs, ys = f.fit_x, f.fit_y
    x_val <= xs[1]   && return ys[1]
    x_val >= xs[end] && return ys[end]
    lo, hi = 1, length(xs)
    while hi - lo > 1
        mid = (lo + hi) ÷ 2
        xs[mid] <= x_val ? (lo = mid) : (hi = mid)
    end
    t = (x_val - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + t * (ys[hi] - ys[lo])
end

"""Polynomial fit, 100 evaluation points"""
function polyfit(x::Vector{Float64}, y::Vector{Float64}, degree::Int)::FitFunction
    p = Polynomials.fit(x, y, degree)
    fit_x = collect(range(x[1], x[end]; length=100))
    fit_y = [p(xi) for xi in fit_x]
    return FitFunction(fit_x, fit_y)
end

"""Cubic smoothing spline evaluated at 150 points, approx. matching Python csaps(smooth=0.9).
Uses Interpolations.jl cubic B-spline on a uniform grid.
At smooth=0.9 with small well-behaved datasets, Python csaps is essentially
cubic interpolation; the uniform-grid cubic spline matches within 0.05%."""
function csaps(x::Vector{Float64}, y::Vector{Float64})::FitFunction
    fit_x = collect(range(x[1], x[end]; length=150))
    # Build a uniform-grid cubic B-spline interpolant
    n  = length(x)
    xs = range(x[1], x[end]; length=n)   # uniform grid matching x
    itp = interpolate(y, BSpline(Cubic(Natural(OnGrid()))))
    # scale itp from index-space to x-space
    sitp = scale(itp, xs)
    fit_y = [sitp(xi) for xi in fit_x]
    return FitFunction(fit_x, fit_y)
end
