module VogelSIM

include("Fitting.jl")
include("MF52.jl")
include("Panda.jl")
include("TireState.jl")
include("VehicleState.jl")
include("GGV.jl")
include("Trajectory.jl")
include("Vehicle.jl")

export FitFunction, evaluate, polyfit, csaps
export MF52, mf52_Fx, mf52_Fy
export Panda
export TireState, eval_Fx!, eval_Fy!
export StateInput, VehicleState, eval!
export GGV, generate!,
       calc_grip_lim_max_accel, calc_power_lim_max_accel, calc_decel
export Trajectory
export Vehicle, simulate_endurance!, simulate_forwards!, simulate_reverse!

end # module VogelSIM
