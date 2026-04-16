using Pkg
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
    return laptime_1lap
end

main()
