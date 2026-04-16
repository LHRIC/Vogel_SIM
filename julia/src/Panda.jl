# Panda.jl — included into module VogelSIM

using CSV, DataFrames

mutable struct Panda
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
    rpm_range::Vector{Float64}
    torque_curve::Vector{Float64}
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
    p = Panda(
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
        Float64[],  # RPM Range
        Float64[],  # Torque Curve
        38.0/18.0,  # Primay Reduction
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

    # Load torque curve
    df = CSV.read(joinpath(base_dir, "setups", "23_torque_curve.csv"), DataFrame; header=false)
    p.rpm_range   = Float64.(df[:, 1])
    p.torque_curve = Float64.(df[:, 2])

    convert_units!(p)
    return p
end

function convert_units!(p::Panda)
    w_mod = (p.torsional_rigidity - p.tr_nom) / 47.5 * 9.81
    p.total_weight   = p.total_weight * 4.4482216153 + w_mod
    p.total_weight_f = p.total_weight * p.weight_dist_f
    p.total_weight_r = p.total_weight * (1.0 - p.weight_dist_f)
    p.cg_height      = p.cg_height   / 39.37
    p.trackwidth_f   = p.trackwidth_f * 0.0254
    p.trackwidth_r   = p.trackwidth_r * 0.0254
    p.trackwidth_max = max(p.trackwidth_f, p.trackwidth_r)
    p.k_phi_f        = p.k_phi_f * 57.2958
    p.k_phi_r        = p.k_phi_r * 57.2958
end
