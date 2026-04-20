module VogelSIM

include("Fitting.jl")
include("MF52.jl")
include("Params.jl")
include("TireState.jl")
include("VehicleState.jl")
include("GGV.jl")
include("Trajectory.jl")
include("TrackPath.jl")
include("Vehicle.jl")

export FitFunction, evaluate, polyfit, csaps
export MF52, mf52_Fx, mf52_Fy
export json_schema
export TorqueCurve, max_throttle_torque
export Panda, Params, convert_units!
export TireState, eval_Fx!, eval_Fy!
export StateInput, VehicleState, eval!
export GGV, generate!,
       calc_grip_lim_max_accel, calc_power_lim_max_accel, calc_decel
export Trajectory
export SplineCurve, Gate, make_gate, Path, TrackPath, build_path, to_trajectory
export Vehicle, simulate_endurance!, simulate_forwards!, simulate_reverse!

end # module VogelSIM
