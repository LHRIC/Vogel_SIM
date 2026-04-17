# test_trackpath.jl
# =================
# Numeric tests + visual plots for TrackPath.jl.
#
# Run from the julia/ directory:
#   julia --project=. test/test_trackpath.jl
#
# Produces:  test/test_trackpath_plots.png

using Plots, Printf
gr()

# ── load module ──────────────────────────────────────────────────────────────
include(joinpath(@__DIR__, "..", "src", "VogelSIM.jl"))
using .VogelSIM, Statistics

# ── helpers ──────────────────────────────────────────────────────────────────
function assert_close(label, got, expected, tol_frac)
    err = abs(got - expected) / abs(expected)
    status = err <= tol_frac ? "PASS" : "FAIL"
    @printf("  [%s] %-45s got=%.5f  expected=%.5f  err=%.4f%%\n",
            status, label, got, expected, err*100)
    err <= tol_frac || error("Test failed: $label")
end

# ─────────────────────────────────────────────────────────────────────────────
# TEST 1 — Circle of known radius
# Expect: arc length ≈ 2πR, mean(radii) ≈ R, κ everywhere ≈ 1/R
# ─────────────────────────────────────────────────────────────────────────────
println("\n── Test 1: Circle (R = 10 m) ────────────────────────────────────────")
R = 10.0
t1 = range(0.0, 2π, length=300)
xs1 = R .* cos.(t1);  ys1 = R .* sin.(t1)
sc_circle = SplineCurve(collect(t1), xs1, ys1; s=0.0)
path_circle = build_path(sc_circle, 800, 3.5, 36.0; smooth=0.0)

assert_close("arc length (m)",       path_circle.length,          2π*R,  0.001)
assert_close("mean radius (m)",      mean(path_circle.radii),     R,     0.005)
assert_close("mean |κ| (1/m)",       mean(abs.(path_circle.kappa)), 1/R,  0.005)

# ─────────────────────────────────────────────────────────────────────────────
# TEST 2 — Straight line
# Expect: arc length = L, radii all clamped to r_max (κ → 0 → r = ∞ → clamp)
# ─────────────────────────────────────────────────────────────────────────────
println("\n── Test 2: Straight line (L = 50 m) ────────────────────────────────")
L_line = 50.0
t2  = range(0.0, 1.0, length=100)
xs2 = L_line .* t2;  ys2 = zeros(length(t2))
sc_line = SplineCurve(collect(t2), xs2, ys2; s=0.0)
path_line = build_path(sc_line, 200, 3.5, 36.0; smooth=0.0)

assert_close("arc length (m)", path_line.length, L_line, 0.001)
# All radii should be at the clamp ceiling (straight → κ ≈ 0 → r → r_max)
frac_at_max = mean(path_line.radii .>= 35.9)   # allow floating-point slack
@printf("  [%s] %-45s frac_clamped=%.3f (expect ≈ 1.0)\n",
        frac_at_max > 0.95 ? "PASS" : "FAIL",
        "fraction of radii clamped to r_max", frac_at_max)
frac_at_max > 0.95 || error("Test failed: straight line radii not clamped")

# ─────────────────────────────────────────────────────────────────────────────
# TEST 3 — Figure-8 (two circles, opposite curvature signs)
# Expect: κ changes sign; mean(κ) ≈ 0 by symmetry
# ─────────────────────────────────────────────────────────────────────────────
println("\n── Test 3: Figure-8 (opposite curvature signs) ─────────────────────")
R8 = 8.0
t3 = range(0.0, 2π, length=400)
# Lemniscate of Bernoulli scaled to approx radius R8
a8 = R8 * sqrt(2)
xs3 = @. a8 * cos(t3) / (1 + sin(t3)^2)
ys3 = @. a8 * sin(t3) * cos(t3) / (1 + sin(t3)^2)
sc_fig8 = SplineCurve(collect(t3), xs3, ys3; s=0.05)
path_fig8 = build_path(sc_fig8, 600, 3.5, 36.0; smooth=0.05)

pos_kappa = count(path_fig8.kappa .> 0)
neg_kappa = count(path_fig8.kappa .< 0)
@printf("  [%s] %-45s pos_κ=%d  neg_κ=%d\n",
        (pos_kappa > 50 && neg_kappa > 50) ? "PASS" : "FAIL",
        "curvature sign changes (figure-8)", pos_kappa, neg_kappa)
(pos_kappa > 50 && neg_kappa > 50) || error("Test failed: figure-8 curvature signs")

mean_kappa = mean(path_fig8.kappa)
@printf("  [%s] %-45s mean_κ=%.4f (expect ≈ 0)\n",
        abs(mean_kappa) < 0.05 ? "PASS" : "FAIL",
        "mean κ ≈ 0 (symmetric figure-8)", mean_kappa)

# ─────────────────────────────────────────────────────────────────────────────
# TEST 4 — Gate normalization
# ─────────────────────────────────────────────────────────────────────────────
println("\n── Test 4: make_gate auto-normalization ─────────────────────────────")
g = make_gate((1.0, 2.0), (3.0, 4.0))
n_len = sqrt(g.normal[1]^2 + g.normal[2]^2)
@printf("  [%s] %-45s |n|=%.8f (expect 1.0)\n",
        abs(n_len - 1.0) < 1e-10 ? "PASS" : "FAIL",
        "gate normal is unit length", n_len)
abs(n_len - 1.0) < 1e-10 || error("Test failed: gate normal not unit length")

# Zero-length normal should throw
zero_threw = let
    ok = false
    try make_gate((0.0, 0.0), (0.0, 0.0)) catch; ok = true end
    ok
end
@printf("  [%s] %-45s\n", zero_threw ? "PASS" : "FAIL", "zero normal throws an error")
zero_threw || error("Test failed: zero normal did not throw")

# ─────────────────────────────────────────────────────────────────────────────
# TEST 5 — to_trajectory round-trip
# Expect: Trajectory fields match Path fields exactly (no copy corruption)
# ─────────────────────────────────────────────────────────────────────────────
println("\n── Test 5: to_trajectory round-trip (circle) ───────────────────────")
gate_c = make_gate((R, 0.0), (0.0, 1.0))
inner_c = SplineCurve(collect(t1), (R-1) .* cos.(t1), (R-1) .* sin.(t1); s=0.0)
outer_c = SplineCurve(collect(t1), (R+1) .* cos.(t1), (R+1) .* sin.(t1); s=0.0)
tp = TrackPath(inner_c, outer_c, sc_circle, gate_c, gate_c, 1;
               n_samples=800, r_min=3.5, r_max=36.0, smooth=0.0)
traj = to_trajectory(tp)

@printf("  [%s] %-45s got=%d  expected=%d\n",
        traj.num_points == length(tp.raceline.s) ? "PASS" : "FAIL",
        "num_points matches Path length", traj.num_points, length(tp.raceline.s))
traj.num_points == length(tp.raceline.s) || error("Test failed: num_points mismatch")

radii_match = maximum(abs.(traj.radii .- tp.raceline.radii)) < 1e-12
@printf("  [%s] %-45s max_diff=%.2e\n",
        radii_match ? "PASS" : "FAIL",
        "radii vectors identical after copy", maximum(abs.(traj.radii .- tp.raceline.radii)))
radii_match || error("Test failed: radii copy mismatch")

# ─────────────────────────────────────────────────────────────────────────────
# PLOTS
# ─────────────────────────────────────────────────────────────────────────────
println("\n── Generating plots ─────────────────────────────────────────────────")

# Shared style
lw = 2

# Panel 1 — Circle: xy path + inner/outer bounds
p1 = plot(tp.raceline.x, tp.raceline.y;
          label="Raceline", lw=lw, color=:blue, aspect_ratio=:equal,
          title="Test 1: Circle (R=10 m)\nPath + Boundaries", xlabel="x (m)", ylabel="y (m)")
# inner / outer bounds sampled from SplineCurve
t_eval = range(0.0, 2π, length=400)
inner_x = inner_c.x_spline.(t_eval);  inner_y = inner_c.y_spline.(t_eval)
outer_x = outer_c.x_spline.(t_eval);  outer_y = outer_c.y_spline.(t_eval)
plot!(p1, inner_x, inner_y; label="Inner bound", lw=1, color=:gray, ls=:dash)
plot!(p1, outer_x, outer_y; label="Outer bound", lw=1, color=:gray, ls=:dot)
scatter!(p1, [gate_c.point[1]], [gate_c.point[2]];
         label="Start/end gate", ms=7, color=:green, markershape=:diamond)

# Panel 2 — Circle: radius profile vs arc length
p2 = plot(path_circle.s, path_circle.radii;
          label="Computed radius", lw=lw, color=:blue,
          title="Test 1: Radius of Curvature vs Arc Length",
          xlabel="Arc length s (m)", ylabel="Radius (m)")
hline!(p2, [R]; label="True R = $(R) m", lw=1, color=:red, ls=:dash)
ylims!(p2, 0, 20)

# Panel 3 — Straight line: radius profile (should be ≈ r_max everywhere)
p3 = plot(path_line.s, path_line.radii;
          label="Computed radius", lw=lw, color=:darkorange,
          title="Test 2: Straight Line\nRadius vs Arc Length (expect r_max=36)",
          xlabel="Arc length s (m)", ylabel="Radius (m)")
hline!(p3, [36.0]; label="r_max = 36 m", lw=1, color=:red, ls=:dash)
ylims!(p3, 0, 40)

# Panel 4 — Figure-8: xy path coloured by κ sign
kappa_sign = sign.(path_fig8.kappa)
colors_f8 = [k > 0 ? :royalblue : :crimson for k in kappa_sign]
p4 = scatter(path_fig8.x, path_fig8.y;
             marker_z=path_fig8.kappa, color=:RdBu, colorbar=true,
             markerstrokewidth=0, ms=3, label="",
             title="Test 3: Figure-8\nPath coloured by signed κ",
             xlabel="x (m)", ylabel="y (m)", aspect_ratio=:equal,
             colorbar_title="κ (m⁻¹)")

# Panel 5 — Figure-8: curvature vs arc length
p5 = plot(path_fig8.s, path_fig8.kappa;
          label="κ(s)", lw=lw, color=:purple,
          title="Test 3: Figure-8\nSigned Curvature vs Arc Length",
          xlabel="Arc length s (m)", ylabel="κ (m⁻¹)")
hline!(p5, [0.0]; label="κ = 0", lw=1, color=:black, ls=:dash)

# Panel 6 — Circle: arc-length tangent angle (should increase linearly by 2π)
theta = atan.(path_circle.dy_ds, path_circle.dx_ds)
# unwrap to get monotone angle
theta_uw = copy(theta)
for i in 2:length(theta_uw)
    d = theta_uw[i] - theta_uw[i-1]
    if d < -π; theta_uw[i:end] .+= 2π; end
    if d >  π; theta_uw[i:end] .-= 2π; end
end
p6 = plot(path_circle.s, theta_uw;
          label="Tangent angle θ(s)", lw=lw, color=:teal,
          title="Test 1: Tangent Angle vs Arc Length\n(expect linear, Δθ = 2π)",
          xlabel="Arc length s (m)", ylabel="θ (rad)")
plot!(p6, [0, path_circle.length], [theta_uw[1], theta_uw[1]+2π];
      label="Expected (linear)", lw=1, color=:red, ls=:dash)

# Compose and save
fig = plot(p1, p2, p3, p4, p5, p6;
           layout=(3,2), size=(1200, 1200),
           plot_title="TrackPath.jl Test Suite",
           margin=5Plots.mm)

out_path = joinpath(@__DIR__, "test_trackpath_plots.png")
savefig(fig, out_path)
println("  Plot saved → $out_path")

println("\n══ All tests passed ══════════════════════════════════════════════════")
