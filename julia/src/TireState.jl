# TireState.jl — included into module VogelSIM
# Depends on: MF52 (mf52_Fx, mf52_Fy)

mutable struct TireState
    mf52::MF52
    friction_scaling_x::Float64
    friction_scaling_y::Float64
    alpha::Float64    # slip angle (rad)
    kappa::Float64    # longitudinal slip
    epsilon::Float64  # inclination angle (rad)
    Fz::Float64       # normal load (N)
    Fx::Float64       # longitudinal force (N)
    Fy::Float64       # lateral force (N)
end

function TireState(mf52::MF52, fsx::Float64, fsy::Float64)
    return TireState(mf52, fsx, fsy, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
end

"""Peak longitudinal force by scanning kappa ∈ [0, 0.2] at 100 steps."""
function eval_Fx!(t::TireState)
    best = 0.0
    for sl in range(0.0, 0.2; length=100)
        val = mf52_Fx(t.mf52, t.Fz, sl, t.epsilon) * t.friction_scaling_x
        val > best && (best = val)
    end
    t.Fx = best
end

function eval_Fy!(t::TireState)
    t.Fy = mf52_Fy(t.mf52, t.Fz, t.alpha, t.epsilon) * t.friction_scaling_y
end
