# MF52.jl — included into module VogelSIM

using MAT

struct MF52
    Fx_params::Vector{Float64}
    Fy_params::Vector{Float64}
    Fz0::Float64
end

function MF52(base_dir::String)
    fx_mat = matread(joinpath(base_dir, "utilities", "18.0x6.0-10_R20_DriveBrakeComb.mat"))
    fy_mat = matread(joinpath(base_dir, "utilities", "16x7.5-10_R20_Cornering.mat"))
    return MF52(vec(fx_mat["x0"]), vec(fy_mat["x0"]), 800.0)
end

function mf52_Fx(tm::MF52, Fz::Float64, Kappa::Float64, Gamma::Float64)::Float64
    p   = tm.Fx_params
    Fz0 = tm.Fz0

    PCX1=p[1]; PDX1=p[2]; PDX2=p[3]; PDX3=p[4]
    PEX1=p[5]; PEX2=p[6]; PEX3=p[7]; PEX4=p[8]
    PKX1=p[9]; PKX2=p[10]; PKX3=p[11]
    PHX1=p[12]; PHX2=p[13]
    PVX1=p[14]; PVX2=p[15]

    dfz   = (Fz - Fz0) / Fz0
    SVx   = Fz * (PVX1 + PVX2 * dfz)
    SHx   = PHX1 + PHX2 * dfz
    KappaX = Kappa + SHx
    Mux   = (PDX1 + PDX2 * dfz) * (1.0 - PDX3 * Gamma^2)
    Cx    = PCX1
    Dx    = Mux * Fz
    Ex    = (PEX1 + PEX2 * dfz + PEX3 * dfz^2) * (1.0 - PEX4 * sign(KappaX))
    Kx_s  = Fz * (PKX1 + PKX2 * dfz) * exp(PKX3 * dfz)
    Bx    = Kx_s / (Cx * Dx)
    return Dx * sin(Cx * atan(Bx*KappaX - Ex*(Bx*KappaX - atan(Bx*KappaX)))) + SVx
end

function mf52_Fy(tm::MF52, Fz::Float64, Alpha_rad::Float64, Gamma::Float64)::Float64
    Alpha = rad2deg(Alpha_rad)   # MF5.2 expects degrees
    p     = tm.Fy_params
    Fz0   = tm.Fz0

    PCY1=p[1]; PDY1=p[2]; PDY2=p[3]; PDY3=p[4]
    PEY1=p[5]; PEY2=p[6]; PEY3=p[7]; PEY4=p[8]
    # p[9] = PEY5 (unused)
    PKY1=p[10]; PKY2=p[11]; PKY3=p[12]
    # p[13..16] = PKY4..PKY7 (unused)
    PHY1=p[17]; PHY2=p[18]
    PVY1=p[19]; PVY2=p[20]; PVY3=p[21]; PVY4=p[22]
    # p[23..27] = PPY1..PPY5 (unused)

    dfz    = (Fz - Fz0) / Fz0
    SVy    = Fz * ((PVY1 + PVY2*dfz) + (PVY3 + PVY4*dfz) * Gamma)
    SHy    = PHY1 + PHY2 * dfz
    AlphaY = Alpha + SHy
    Muy    = (PDY1 + PDY2*dfz) * (1.0 - PDY3*Gamma^2)
    Cy     = PCY1
    Dy     = Muy * Fz
    Ey     = (PEY1 + PEY2*dfz) * (1.0 - (PEY3 + PEY4*Gamma)*sign(AlphaY))
    Ky     = PKY1*Fz0 * sin(2.0*atan(Fz/(PKY2*Fz0))) * (1.0 - PKY3*abs(Gamma))
    By     = Ky / (Cy * Dy)
    return Dy * sin(Cy * atan(By*AlphaY - Ey*(By*AlphaY - atan(By*AlphaY)))) + SVy
end
