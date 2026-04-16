# VehicleState.jl — included into module VogelSIM
# Depends on: MF52, TireState, Panda

struct StateInput
    Ax::Float64; Ay::Float64; v::Float64
    r::Float64; delta::Float64; beta::Float64
end
StateInput(; Ax=0.0, Ay=0.0, v=0.0, r=0.0, delta=0.0, beta=0.0) =
    StateInput(Ax, Ay, v, r, delta, beta)

mutable struct VehicleState
    # Mirrored scalar params (avoid re-allocating Panda in hot loops)
    total_weight_f::Float64;
    total_weight_r::Float64;
    total_weight::Float64
    cg_height::Float64;
    rollc_f::Float64;
    rollc_r::Float64
    trackwidth_f::Float64;
    trackwidth_r::Float64
    k_phi_f::Float64;
    k_phi_r::Float64
    ride_rate_f::Float64;
    ride_rate_r::Float64
    weight_dist_f::Float64;
    wheelbase::Float64
    asquat::Float64;
    adive::Float64
    CoP::Float64;
    Cl::Float64
    static_camber_f::Float64;
    static_camber_r::Float64
    camber_gain_f::Float64;
    camber_gain_r::Float64
    friction_scaling_x::Float64;
    friction_scaling_y::Float64
    # Live state
    v::Float64;
    r::Float64;
    Ax::Float64;
    Ay::Float64
    delta::Float64;
    beta::Float64
    phi::Float64;
    theta::Float64;
    dz_CG::Float64;
    cgz::Float64
    alpha_f::Float64;
    alpha_r::Float64;
    downforce::Float64
    fl_tire::TireState;
    fr_tire::TireState
    rl_tire::TireState;
    rr_tire::TireState
end

function VehicleState(mf52::MF52, p::Panda)
    ts() = TireState(mf52, p.friction_scaling_x, p.friction_scaling_y)
    VehicleState(
        p.total_weight_f, p.total_weight_r, p.total_weight,
        p.cg_height, p.rollc_f, p.rollc_r,
        p.trackwidth_f, p.trackwidth_r,
        p.k_phi_f, p.k_phi_r, p.ride_rate_f, p.ride_rate_r,
        p.weight_dist_f, p.wheelbase, p.asquat, p.adive,
        p.CoP, p.Cl,
        p.static_camber_f, p.static_camber_r,
        p.camber_gain_f,   p.camber_gain_r,
        p.friction_scaling_x, p.friction_scaling_y,
        0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
        ts(), ts(), ts(), ts()
    )
end

function eval!(vs::VehicleState, si::StateInput)
    vs.v=si.v; vs.r=si.r; vs.Ax=si.Ax; vs.Ay=si.Ay
    vs.delta=si.delta; vs.beta=si.beta

    m_f=vs.total_weight_f; m_r=vs.total_weight_r; m_t=vs.total_weight

    kφ = vs.k_phi_f / (vs.k_phi_f + vs.k_phi_r)
    rl_f = vs.cg_height - vs.rollc_f
    rl_r = vs.cg_height - vs.rollc_r

    dFz_geom_roll_f = m_f * vs.rollc_f * vs.Ay / vs.trackwidth_f
    dFz_geom_roll_r = m_r * vs.rollc_r * vs.Ay / vs.trackwidth_r
    dFz_elas_roll_f = kφ * (m_f*rl_f + m_r*rl_r) * vs.Ay / vs.trackwidth_f
    dFz_elas_roll_r = (1.0-kφ) * (m_f*rl_f + m_r*rl_r) * vs.Ay / vs.trackwidth_r

    dFz_tot_pitch = vs.cg_height * m_t * vs.Ax / vs.wheelbase
    if dFz_tot_pitch >= 0.0
        dFz_elas_pitch_r =  dFz_tot_pitch * (1.0 - vs.asquat)
        dFz_elas_pitch_f = -dFz_tot_pitch
    else
        dFz_elas_pitch_r =  dFz_tot_pitch
        dFz_elas_pitch_f = -dFz_tot_pitch * (1.0 - vs.adive)
    end
    dFz_geom_pitch_r = dFz_tot_pitch - dFz_elas_pitch_r
    dFz_geom_pitch_f = -dFz_tot_pitch + dFz_elas_pitch_f

    dFz_aero_f = 1.293/2.0 * vs.CoP * vs.Cl * vs.v^2
    dFz_aero_r = 1.293/2.0 * (1.0-vs.CoP) * vs.Cl * vs.v^2
    vs.downforce = dFz_aero_f + dFz_aero_r

    Fz_elas_FL = -dFz_elas_roll_f + dFz_elas_pitch_f/2.0 + dFz_aero_f/2.0
    Fz_elas_FR =  dFz_elas_roll_f + dFz_elas_pitch_f/2.0 + dFz_aero_f/2.0
    Fz_elas_RL = -dFz_elas_roll_r + dFz_elas_pitch_r/2.0 + dFz_aero_r/2.0
    Fz_elas_RR =  dFz_elas_roll_r + dFz_elas_pitch_r/2.0 + dFz_aero_r/2.0

    Fz_tot_FL = m_f/2.0 + Fz_elas_FL - dFz_geom_roll_f + dFz_geom_pitch_f/2.0
    Fz_tot_FR = m_f/2.0 + Fz_elas_FR + dFz_geom_roll_f + dFz_geom_pitch_f/2.0
    Fz_tot_RL = m_r/2.0 + Fz_elas_RL - dFz_geom_roll_r + dFz_geom_pitch_r/2.0
    Fz_tot_RR = m_r/2.0 + Fz_elas_RR + dFz_geom_roll_r + dFz_geom_pitch_r/2.0

    dz_FL = Fz_elas_FL / (vs.ride_rate_f/1000.0)
    dz_FR = Fz_elas_FR / (vs.ride_rate_f/1000.0)
    dz_RL = Fz_elas_RL / (vs.ride_rate_r/1000.0)
    dz_RR = Fz_elas_RR / (vs.ride_rate_r/1000.0)

    cam_FL = vs.static_camber_f + dz_FL*vs.camber_gain_f
    cam_FR = vs.static_camber_f + dz_FR*vs.camber_gain_f
    cam_RL = vs.static_camber_r + dz_RL*vs.camber_gain_r
    cam_RR = vs.static_camber_r + dz_RR*vs.camber_gain_r

    phi_f = rad2deg(asin((dz_FR-dz_FL)/(vs.trackwidth_f*1000.0)))
    phi_r = rad2deg(asin((dz_RR-dz_RL)/(vs.trackwidth_r*1000.0)))
    vs.phi = (phi_f+phi_r)/2.0

    IA_FL = deg2rad(cam_FL - vs.phi)
    IA_FR = deg2rad(cam_FR + vs.phi)
    IA_RL = deg2rad(cam_RL - vs.phi)
    IA_RR = deg2rad(cam_RR + vs.phi)

    vs.theta = rad2deg(asin(((dz_FR+dz_FL)/2.0-(dz_RR+dz_RL)/2.0)/(vs.wheelbase*1000.0)))
    vs.dz_CG = (((dz_FR+dz_FL)/2.0)-((dz_RR+dz_RL)/2.0))*vs.weight_dist_f + ((dz_RR+dz_RL)/2.0)
    vs.cgz   = -vs.dz_CG/1000.0 + vs.cg_height

    vs.fl_tire.Fz=Fz_tot_FL; vs.fr_tire.Fz=Fz_tot_FR
    vs.rl_tire.Fz=Fz_tot_RL; vs.rr_tire.Fz=Fz_tot_RR
    vs.fl_tire.epsilon=IA_FL; vs.fr_tire.epsilon=IA_FR
    vs.rl_tire.epsilon=IA_RL; vs.rr_tire.epsilon=IA_RR

    a = vs.wheelbase*(1.0-vs.weight_dist_f)
    b = vs.wheelbase*vs.weight_dist_f
    if vs.r == 0.0
        vs.alpha_f = 0.0; vs.alpha_r = 0.0
    else
        vs.alpha_f = vs.beta + a/vs.r - vs.delta
        vs.alpha_r = vs.beta - b/vs.r
    end

    vs.fl_tire.alpha=vs.alpha_f; vs.fr_tire.alpha=vs.alpha_f
    vs.rl_tire.alpha=vs.alpha_r; vs.rr_tire.alpha=vs.alpha_r

    eval_Fx!(vs.fl_tire); eval_Fx!(vs.fr_tire)
    eval_Fx!(vs.rl_tire); eval_Fx!(vs.rr_tire)
    eval_Fy!(vs.fl_tire); eval_Fy!(vs.fr_tire)
    eval_Fy!(vs.rl_tire); eval_Fy!(vs.rr_tire)
end
