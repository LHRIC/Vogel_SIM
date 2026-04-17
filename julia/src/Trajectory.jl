using CSV, DataFrames

struct Trajectory
    points::Tuple{Vector{Float64},Vector{Float64}}
    radii::Vector{Float64}
    _curvature::Vector{Float64}
    num_points::Int
end

"""
Load X,Y from a *_ft.csv file (converting feet→metres).
Radii and curvature are read from the pre-computed trackmaps folder:
  <same_dir>/trackmaps/<stem>_radii_m.csv
  <same_dir>/trackmaps/<stem>_curvature_m.csv
Radii are clamped to [r_min, r_max].
"""
json_schema(::Type{Trajectory}) = Dict(
    "type"        => "Object",
    "required"    => true,
    "description" => "Track geometry (SI units)",
    "shape"       => Dict(
        "x"         => "Array<Float64> — X coordinates (m)",
        "y"         => "Array<Float64> — Y coordinates (m)",
        "radii"     => "Array<Float64> — corner radii (m)",
        "curvature" => "Array<Float64> — signed curvature (1/m)",
    ),
)

function Trajectory(x_m::Vector{Float64}, y_m::Vector{Float64},
                    radii::Vector{Float64}, curvature::Vector{Float64},
                    r_min::Float64, r_max::Float64)
    clamped = clamp.(radii, r_min, r_max)
    n = length(clamped)
    return Trajectory((x_m, y_m), clamped, curvature, n)
end

function Trajectory(file::String, ::Bool, r_min::Float64, r_max::Float64)
    df    = CSV.read(file, DataFrame)
    X_m   = Float64.(df[!, "X"]) .* 0.3048
    Y_m   = Float64.(df[!, "Y"]) .* 0.3048

    dir   = dirname(file)
    stem  = replace(basename(file), r"_ft\.csv$"i => "")
    tdir  = joinpath(dir, "trackmaps")

    radii_df = CSV.read(joinpath(tdir, stem*"_radii_m.csv"),     DataFrame)
    curv_df  = CSV.read(joinpath(tdir, stem*"_curvature_m.csv"), DataFrame)

    radii = clamp.(Float64.(radii_df[!, "radii"]),     r_min, r_max)
    curv  = Float64.(curv_df[!, "curvature"])

    n = length(radii)
    return Trajectory((X_m, Y_m), radii, curv, n)
end
