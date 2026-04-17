using Pkg
using Plots
Pkg.activate(@__DIR__)

include("src/VogelSIM.jl")
using .VogelSIM

function main()
    base_dir   = joinpath(@__DIR__, "..")
    trajectory = joinpath(base_dir, "trajectory", "23_michigan_endurance_ft.csv")

    println("Loading vehicle parameters...")
    params = Panda(base_dir)

    println("Building vehicle & trajectory...")
    vehicle = Vehicle(params, trajectory, true; base_dir=base_dir)

    println("Generating GGV envelope...")
    generate!(vehicle.ggv)

    println("Running lap simulation...")
    laptime_1lap = simulate_endurance!(vehicle)

    laptime = laptime_1lap * 10.0
    Tmax    = 1973.419; Tmin = 1360.978
    score   = 250.0 * ((Tmax/laptime) - 1.0) / ((Tmax/Tmin) - 1.0) + 25.0

    println("Laptime (1 lap):   $(round(laptime_1lap; digits=3)) s")
    println("Laptime (10 laps): $(round(laptime;     digits=3)) s")
    println("Score:             $(round(score;       digits=3))")
    

    return laptime_1lap, vehicle

end

laptime::Float64, vehicle::Vehicle = main()
p1 = plot(vehicle.x,vehicle.y)

@userplot TrackPlot
@recipe function f(cp::TrackPlot)
    x, y, i = cp.args
    n = length(x)
    inds = circshift(1:n, 1 - i)
    linewidth --> range(0, 10, n)
    seriesalpha --> range(0, 1, n)
    aspect_ratio --> 1
    label --> false
    x[inds], y[inds]
end

anim = @animate for i ∈ 1:size(vehicle.time)
    trackplot(vehicle.x, vehicle.y, i)
end

gif(anim, "anim_fps30.gif", fps = 30)

display(p)