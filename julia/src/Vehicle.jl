mutable struct Vehicle
    params::Params
    mf52::MF52
    gear_tot::Float64
    v_max::Float64
    ggv::GGV
    trajectory::Trajectory
    track_path::Union{TrackPath,Nothing}  # retained for future path queries; nothing when loaded from CSV
    _interval::Int
    _mesh_size::Int
    # Mesh arrays
    count::Vector{Float64}
    time::Vector{Float64}
    x::Vector{Float64}; y::Vector{Float64}
    dist::Vector{Float64}; dist_f::Vector{Float64}; dist_r::Vector{Float64}
    turn_dir::Vector{Float64}
    velocity::Vector{Float64}; velocity_f::Vector{Float64}; velocity_r::Vector{Float64}
    gear::Vector{Float64}; is_shifting::Vector{Float64}
    ax::Vector{Float64}; ax_f::Vector{Float64}; ax_r::Vector{Float64}
    ay::Vector{Float64}; ay_f::Vector{Float64}; ay_r::Vector{Float64}
    # Post-processed
    cgz::Vector{Float64}; roll::Vector{Float64}; pitch::Vector{Float64}
end

"""
    Vehicle(params, track; base_dir, calc_lateral, mesh_resolution)

Construct a `Vehicle` from a `TrackPath`. The `TrackPath` is converted to a
`Trajectory` internally so the lap simulation runs unchanged. The original
`TrackPath` is stored on the vehicle for future use (track-width queries, etc.).
"""
function _vehicle_from_parts(params::Params, mf52::MF52, traj::Trajectory;
                             calc_lateral::Bool=true, mesh_resolution::Int=10)
    gear_tot = params.gear_ratios[end] * params.final_drive * params.primary_reduction
    v_max    = params.shiftpoint / (gear_tot/params.tire_radius * 60.0/(2π))
    ggv      = GGV(params, mf52, gear_tot, v_max; calc_lateral=calc_lateral)
    generate!(ggv)
    iv       = mesh_resolution
    msz      = (traj.num_points - 1) * iv
    z()      = zeros(msz)
    Vehicle(params, mf52, gear_tot, v_max, ggv, traj, nothing, iv, msz,
            z(),z(),z(),z(), z(),z(),z(), z(),
            z(),z(),z(), z(),z(), z(),z(),z(), z(),z(),z(),
            Float64[], Float64[], Float64[])
end

function Vehicle(params::Params, trajectory_path::String, is_closed::Bool;
                 base_dir::String=".", calc_lateral::Bool=true, mesh_resolution::Int=10)
    mf52 = MF52(base_dir)
    traj = Trajectory(trajectory_path, is_closed, 3.5, 36.0)
    return _vehicle_from_parts(params, mf52, traj;
                               calc_lateral=calc_lateral, mesh_resolution=mesh_resolution)
end

function Vehicle(params::Params, mf52::MF52, traj::Trajectory;
                 calc_lateral::Bool=true, mesh_resolution::Int=10)
    return _vehicle_from_parts(params, mf52, traj;
                               calc_lateral=calc_lateral, mesh_resolution=mesh_resolution)
end

# ── Kinematic helper ─────────────────────────────────────────────
function solve_dt(delta_d::Float64, vel::Float64, ax::Float64)::Float64
    a_c = 0.5*9.81*ax; b_c = vel; c_c = -delta_d
    abs(a_c) < 1e-12 && return delta_d/vel
    disc = b_c^2 - 4.0*a_c*c_c
    disc < 0.0 && return delta_d/vel
    sq = sqrt(disc)
    t1 = (-b_c+sq)/(2.0*a_c); t2 = (-b_c-sq)/(2.0*a_c)
    cands = filter(t->t>0.0, [t1,t2])
    isempty(cands) && return delta_d/vel
    return maximum(cands)
end

# ── Forward pass ─────────────────────────────────────────────────
function simulate_forwards!(v::Vehicle, starting_vel::Float64)
    vel=starting_vel; gear=1; req_gear=1
    t_shift=0.0; shifting=false; cnt=0; dist_acc=0.0; time=0.0

    for pi in 1:(v.trajectory.num_points-1)
        x1=v.trajectory.points[1][pi]; y1=v.trajectory.points[2][pi]
        x2=v.trajectory.points[1][pi+1]; y2=v.trajectory.points[2][pi+1]
        seg = sqrt((x1-x2)^2+(y1-y2)^2)
        r   = v.trajectory.radii[pi]
        v_max = min(v.v_max, evaluate(v.ggv.cornering_capability, r))

        for (ei,vv) in enumerate(v.ggv.velocity_range)
            if Float64(vv) > vel; req_gear = v.ggv.expected_gears[ei]; break; end
        end
        shifting = (req_gear != gear)

        AX_cap  = evaluate(v.ggv.accel_capability,  vel)
        AY_cap  = evaluate(v.ggv.lateral_capability, vel)
        AY_act  = vel^2/(r*9.81)
        ddelta  = seg/v._interval

        for j in 0:(v._interval-1)
            idx = cnt+1
            v.x[idx]=x1+(x2-x1)*j/v._interval; v.y[idx]=y1+(y2-y1)*j/v._interval
            v.ay_f[idx] = min(AY_cap, AY_act)
            v.turn_dir[idx] = sign(v.trajectory._curvature[pi])
            v.count[idx]    = Float64(cnt)
            v.velocity_f[idx] = vel
            v.gear[idx]     = Float64(gear)
            v.dist_f[idx]   = dist_acc + ddelta*j
            dt = ddelta/vel
            v.time[idx] = time + dt*j

            if shifting && vel < v_max
                t_shift += ddelta/vel; v.ax_f[idx]=0.0; v.is_shifting[idx]=1.0
            elseif vel < v_max
                ax_pot = AX_cap * sqrt(1.0-(min(AY_cap,AY_act)/AY_cap)^2)
                v.ax_f[idx] = ax_pot
                dt  = solve_dt(ddelta, vel, ax_pot)
                vel = min(v_max, vel + 9.81*ax_pot*dt)
                for i in eachindex(v.ggv.expected_gears)
                    Float64(v.ggv.expected_gears[i]) > vel && (req_gear=v.ggv.expected_gears[i])
                end
                shifting = (req_gear > gear)
            else
                vel=v_max; dt=ddelta/vel; v.ax_f[idx]=0.0
            end

            t_shift >= v.params.shift_time && (shifting=false; t_shift=0.0; gear=req_gear)
            cnt+=1; time+=dt
        end
        dist_acc+=seg
    end
end

# ── Reverse pass ─────────────────────────────────────────────────
function simulate_reverse!(v::Vehicle)
    vel=v.velocity_f[end]; dist_acc=0.0
    cnt=Int(v.count[end])                        # 0-based index of last point

    for pi in v.trajectory.num_points:-1:1
        x1=v.trajectory.points[1][pi];  y1=v.trajectory.points[2][pi]
        # wrap-around for closed track: index 0 (Python) → index n (Julia, last)
        pi_prev = pi==1 ? v.trajectory.num_points : pi-1
        x2=v.trajectory.points[1][pi_prev]; y2=v.trajectory.points[2][pi_prev]
        seg = sqrt((x1-x2)^2+(y1-y2)^2)
        r   = v.trajectory.radii[pi]
        v_max = min(v.v_max, evaluate(v.ggv.cornering_capability, r))

        AX_cap  = -1.0*evaluate(v.ggv.braking_capability, vel)
        AY_cap  = evaluate(v.ggv.lateral_capability, vel)
        AY_act  = vel^2/(r*9.81)
        ddelta  = seg/v._interval

        for j in 0:(v._interval-1)
            # Use modular indexing for the extra wrap segment
            idx = mod(cnt, v._mesh_size) + 1    # 1-based, wraps
            v.velocity_r[idx] = vel
            v.dist_r[idx]     = dist_acc + ddelta*j
            v.ay_r[idx]       = min(AY_cap, AY_act)

            if vel < v_max
                AX_act = AX_cap*(1.0-(min(AY_cap,AY_act)/AY_cap)^2)
                v.ax_r[idx] = AX_act
                dt  = solve_dt(ddelta, vel, AX_act)
                dv  = 9.81*AX_act*dt
                vel += min(dv, v_max-vel)
            else
                vel=v_max; v.ax_r[idx]=0.0
            end
            cnt-=1
        end
        dist_acc+=seg
    end
end

# ── Endurance merge ───────────────────────────────────────────────
function simulate_endurance!(v::Vehicle)::Float64
    simulate_forwards!(v, 20.0*0.3048)
    simulate_reverse!(v)
    simulate_forwards!(v, v.velocity_r[1])

    ay_max = evaluate(v.ggv.lateral_capability, v.v_max+1.0)
    for i in eachindex(v.count)
        v.dist[i] = v.dist_f[i]
        if v.velocity_f[i] < v.velocity_r[i]
            v.velocity[i]=v.velocity_f[i]; v.ax[i]=v.ax_f[i]; v.ay[i]=v.ay_f[i]
        else
            v.velocity[i]=v.velocity_r[i]; v.ax[i]=-v.ax_r[i]; v.ay[i]=v.ay_r[i]
        end
    end

    for i in eachindex(v.ay)
        v.ay[i]   > ay_max && (v.ay[i]=ay_max)
        v.ay_f[i] > ay_max && (v.ay_f[i]=ay_max)
        v.ay_r[i] > ay_max && (v.ay_r[i]=ay_max)
    end
    v.ay   .*= v.turn_dir; v.ay_f .*= v.turn_dir; v.ay_r .*= v.turn_dir
    v.ax_r .*= -1.0

    # Post-process vehicle state per trajectory point
    for i in 1:(v.trajectory.num_points-1)
        r  = v.trajectory.radii[i]
        si = StateInput(Ax=v.ax[i], Ay=v.ay[i], v=v.velocity[i], r=r)
        vs = VehicleState(v.mf52, v.params)
        eval!(vs, si)
        push!(v.cgz,  vs.cgz)
        push!(v.roll,  vs.phi)
        push!(v.pitch, vs.theta)
    end

    return maximum(v.time)
end
