using CSV, DataFrames

# Fallback: scalar/primitive types describe themselves by name.
# Files that define composite types add their own json_schema(::Type{T}) methods.
json_schema(T::Type) = Dict("type" => string(T))

struct TorqueCurve
    rpm::Vector{Float64}
    torque::Vector{Float64}
    throttle::Union{Vector{Float64}, Nothing}
end

json_schema(::Type{TorqueCurve}) = Dict(
    "type"        => "Object",
    "required"    => true,
    "description" => "Engine torque map",
    "shape"       => Dict(
        "rpm"      => "Array<Float64>",
        "torque"   => "Array<Float64>",
        "throttle" => "Array<Float64> | null — if present, full-throttle rows are isolated",
    ),
)

# Return the interpolated torque at full (100%) throttle for a given RPM.
# When throttle data is present, linearly interpolates within the subset of
# rows nearest to throttle=1.0 so that partial-throttle maps are handled
# correctly.  Falls back to a plain 1-D interpolation when throttle=nothing.
function max_throttle_torque(tc::TorqueCurve, rpm::Float64)::Float64
    if tc.throttle === nothing
        return _interp1(tc.rpm, tc.torque, rpm)
    end

    # Find indices where throttle == maximum available (full throttle rows)
    max_thr = maximum(tc.throttle)
    mask    = findall(t -> abs(t - max_thr) < 1e-6, tc.throttle)
    return _interp1(tc.rpm[mask], tc.torque[mask], rpm)
end

# Simple linear interpolation (used above and in GGV.jl)
function _interp1(xs::Vector{Float64}, ys::Vector{Float64}, x::Float64)::Float64
    x <= xs[1]   && return ys[1]
    x >= xs[end] && return ys[end]
    lo, hi = 1, length(xs)
    while hi - lo > 1
        mid = (lo + hi) ÷ 2
        xs[mid] <= x ? (lo = mid) : (hi = mid)
    end
    t = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + t * (ys[hi] - ys[lo])
end

mutable struct Params
    total_weight::Float64
    weight_dist_f::Float64
    cg_height::Float64
    wheelbase::Float64
    trackwidth_f::Float64
    trackwidth_r::Float64
    rollc_f::Float64
    rollc_r::Float64
    asquat::Float64
    adive::Float64
    torsional_rigidity::Float64
    tr_nom::Float64
    ride_rate_f::Float64
    ride_rate_r::Float64
    k_phi_f::Float64
    k_phi_r::Float64
    tire_radius::Float64
    static_camber_f::Float64
    static_camber_r::Float64
    camber_gain_f::Float64
    camber_gain_r::Float64
    friction_scaling_x::Float64
    friction_scaling_y::Float64
    Cl::Float64
    Cd::Float64
    CoP::Float64
    torque_curve::TorqueCurve
    primary_reduction::Float64
    gear_ratios::Vector{Float64}
    num_gears::Int
    final_drive::Float64
    shiftpoint::Float64
    drivetrain_losses::Float64
    shift_time::Float64
    diff_locked::Bool
    LLTD::Float64
    # Derived (filled by convert_units!)
    total_weight_f::Float64
    total_weight_r::Float64
    trackwidth_max::Float64
end

function Panda(base_dir::String; overrides::Dict{String,Float64}=Dict{String,Float64}())
    p = Params(
        595.0,      # total weight
        0.46,       # front weight dist
        12.0866,    # cg_height (inch)
        1.535,      # wheelbase (m)
        48.875,     # front track width (inch)
        48.875,     # rear track width (inch)
        0.028,      # front roll center height (m)
        0.036,      # rear roll center height (m)
        0.0,        # anti squat
        0.0,        # anti dive
        1184.0,     # torsional rigidity
        1184.0,     # nominal torsional rigidity
        31430.0,    # front ride rate
        38375.0,    # rear ride rate
        486.0,      # front spring rate
        650.0,      # rear spring rate
        0.2032,     # tire radius
        0.0,        # static front camber
        0.0,        # static rear camber
        0.0,        # front camber gain
        0.0,        # rear camber gain
        0.6,        # x friction scaling
        0.6,        # y friction scaling
        2.8,        # Coefficient of Lift (negative)
        1.1,        # Coefficient of Drag
        0.51,       # Center of Pressure Position
        TorqueCurve(Float64[], Float64[], nothing),  # torque curve (populated below)
        38.0/18.0,  # Primary Reduction
        [33/12, 32/16, 30/18, 26/18, 30/23, 29/24],     # Transmission Gear Ratios
        6,          # Number of Gears
        37.0/11.0,  # Final Drive Ratio
        12500.0,    # Shift point (RPM)
        0.85,       # Drivetrain Transmission Efficiency
        0.25,       # Shift Time (sec)
        false,      # Differential Locked?
        0.5,        # Lateral Load Transfer Dist.
        0.0,        # Derived        
        0.0,        # Derived
        0.0         # Derived
    )

    for (k, v) in overrides
        setfield!(p, Symbol(k), v)
    end

    # Load torque curve from CSV (no throttle column — full-throttle map)
    df = CSV.read(joinpath(base_dir, "setups", "23_torque_curve.csv"), DataFrame; header=false)
    p.torque_curve = TorqueCurve(Float64.(df[:, 1]), Float64.(df[:, 2]), nothing)

    convert_units!(p)
    return p
end

function convert_units!(p::Params)
    p.total_weight   = p.total_weight * 4.4482216153
    p.total_weight_f = p.total_weight * p.weight_dist_f
    p.total_weight_r = p.total_weight * (1.0 - p.weight_dist_f)
    p.cg_height      = p.cg_height   / 39.37
    p.trackwidth_f   = p.trackwidth_f * 0.0254
    p.trackwidth_r   = p.trackwidth_r * 0.0254
    p.trackwidth_max = max(p.trackwidth_f, p.trackwidth_r)
    p.k_phi_f        = p.k_phi_f * 57.2958
    p.k_phi_r        = p.k_phi_r * 57.2958
end
