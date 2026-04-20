# src/api/server.jl - HTTP server wrapping VogelSIM lap simulation
using HTTP
using JSON3
using Dates

include("../VogelSIM.jl")
using .VogelSIM

# ── Schema helpers ────────────────────────────────────────────────────────────

# Build the full /lapsim/args schema by reflecting over Params, then
# dispatching json_schema(T) for each field type.  Any type that defines its
# own json_schema method (TorqueCurve, MF52, Trajectory) gets a rich shape
# description automatically — no hardcoded lists needed here.
function lapsim_args_schema()
    derived = (:total_weight_f, :total_weight_r, :trackwidth_max)

    params_entries = [
        merge(
            Dict("name" => string(f), "source" => "Params",
                 "required" => f ∉ derived),
            json_schema(fieldtype(Params, f)),
        )
        for f in collect(fieldnames(Params))::Vector{Symbol} if f ∉ derived
    ]

    # Trajectory and MF52 are not Params fields but are required by Vehicle.
    vehicle_entries = [
        merge(Dict("name" => "trajectory", "source" => "Vehicle"), json_schema(Trajectory)),
        merge(Dict("name" => "mf52",       "source" => "Vehicle"), json_schema(MF52)),
        Dict("name" => "mesh_resolution", "source" => "Vehicle",
             "type" => "Int", "required" => false, "default" => 10,
             "description" => "Sub-divisions per trajectory segment"),
        Dict("name" => "calc_lateral", "source" => "Vehicle",
             "type" => "Bool", "required" => false, "default" => true,
             "description" => "Solve lateral GGV envelope; false uses precomputed values"),
    ]

    return vcat(params_entries, vehicle_entries)
end

# ── Request parsing helpers ───────────────────────────────────────────────────

function _require(raw, key)
    haskey(raw, key) || error("Missing required field: \"$key\"")
    return raw[key]
end

function _parse_mf52(raw)
    r = _require(raw, "mf52")
    MF52(Float64.(r["fx_params"]), Float64.(r["fy_params"]))
end

function _parse_torque_curve(raw)
    r   = _require(raw, "torque_curve")
    thr = get(r, "throttle", nothing)
    TorqueCurve(Float64.(r["rpm"]), Float64.(r["torque"]),
                thr === nothing ? nothing : Float64.(thr))
end

function _parse_trajectory(raw)
    r = _require(raw, "trajectory")
    Trajectory(Float64.(r["x"]), Float64.(r["y"]),
               Float64.(r["radii"]), Float64.(r["curvature"]),
               3.5, 36.0)
end

function _parse_params(raw, tc::TorqueCurve)
    derived = (:total_weight_f, :total_weight_r, :trackwidth_max)

    vals = map(fieldnames(Params)) do f
        ft = fieldtype(Params, f)
        k  = string(f)
        if f in collect(derived)::Vector{Symbol}
            zero(ft)                                      # filled by convert_units!
        elseif ft === TorqueCurve
            tc
        elseif ft === Bool
            Bool(get(raw, k, false))
        elseif ft === Int
            Int(get(raw, k, f === :num_gears ? 6 : 0))
        elseif ft === Vector{Float64}
            haskey(raw, k) ? Float64.(raw[k]) :
                f === :gear_ratios ? [33/12, 32/16, 30/18, 26/18, 30/23, 29/24] : Float64[]
        else  # Float64
            haskey(raw, k) || error("Missing required field: \"$k\"")
            Float64(raw[k])
        end
    end

    params = Params(vals...)
    convert_units!(params)
    return params
end

# ── Handlers ──────────────────────────────────────────────────────────────────

function handle_root(_::HTTP.Request)
    HTTP.Response(200, "VogelSIM Julia server")
end

function handle_health(_::HTTP.Request)
    HTTP.Response(200, ["Content-Type" => "application/json"], JSON3.write(Dict(
        "status"        => "ok",
        "timestamp"     => string(now()),
        "julia_version" => string(VERSION),
    )))
end

function handle_lapsim_args(_::HTTP.Request)
    HTTP.Response(200, ["Content-Type" => "application/json"],
                  JSON3.write(lapsim_args_schema()))
end

function handle_lapsim(req::HTTP.Request)
    raw = try
        JSON3.read(req.body, Dict{String,Any})
    catch
        return HTTP.Response(400, ["Content-Type" => "application/json"],
            JSON3.write(Dict("error" => "Invalid JSON body")))
    end

    result = try
        mf52   = _parse_mf52(raw)
        tc     = _parse_torque_curve(raw)
        params = _parse_params(raw, tc)
        traj   = _parse_trajectory(raw)

        calc_lateral    = Bool(get(raw, "calc_lateral",    true))
        mesh_resolution = Int(get(raw,  "mesh_resolution", 10))

        vehicle      = Vehicle(params, mf52, traj;
                               calc_lateral=calc_lateral,
                               mesh_resolution=mesh_resolution)
        generate!(vehicle.ggv)
        laptime_1lap = simulate_endurance!(vehicle)
        laptime_10   = laptime_1lap * 10.0

        Tmax  = 1973.419; Tmin = 1360.978
        score = 250.0 * ((Tmax / laptime_10) - 1.0) / ((Tmax / Tmin) - 1.0) + 25.0

        Dict(
            "laptime_1lap_s"  => round(laptime_1lap,  digits=3),
            "laptime_10lap_s" => round(laptime_10,    digits=3),
            "score"           => round(score,         digits=3),
            "v_max_ms"        => round(vehicle.v_max, digits=3),
            "julia_version"   => string(VERSION),
            "profile" => Dict(
                "dist"       => vehicle.dist,
                "time"       => vehicle.time,
                "x"          => vehicle.x,
                "y"          => vehicle.y,
                "velocity"   => vehicle.velocity,
                "ax"         => vehicle.ax,
                "ay"         => vehicle.ay,
                "gear"       => vehicle.gear,
                "is_shifting" => vehicle.is_shifting,
                "turn_dir"   => vehicle.turn_dir,
            ),
            "state_profile" => Dict(
                "cgz"   => vehicle.cgz,
                "roll"  => vehicle.roll,
                "pitch" => vehicle.pitch,
            ),
        )
    catch e
        return HTTP.Response(500, ["Content-Type" => "application/json"],
            JSON3.write(Dict("error" => sprint(showerror, e))))
    end

    HTTP.Response(200, ["Content-Type" => "application/json"], JSON3.write(result))
end

# ── Router ────────────────────────────────────────────────────────────────────

function router(req::HTTP.Request)
    path = HTTP.URI(req.target).path
    if     path == "/"            return handle_root(req)
    elseif path == "/health"      return handle_health(req)
    elseif path == "/lapsim"      return handle_lapsim(req)
    elseif path == "/lapsim/args" return handle_lapsim_args(req)
    else                          return HTTP.Response(404, "Not Found")
    end
end

# ── Entry point ───────────────────────────────────────────────────────────────

function start_server()
    port = parse(Int, get(ENV, "PORT", "8082"))
    println("Starting VogelSIM server on port $port")
    HTTP.serve(router, "0.0.0.0", port)
end

if abspath(PROGRAM_FILE) == @__FILE__
    start_server()
end
