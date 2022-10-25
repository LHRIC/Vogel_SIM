function F = latVehState(delta, beta, A_y, V, W, cg, L, a, b, LLTD, twf, twr, wf, wr, Cd)
    
%     % Calculate turn radius
%     R = V^2/A_y;
% 
%     %calculate yaw rate
%     omega = V/R;
% 
%     %from yaw, sideslip and steer you can get slip angles (rad)
%     a_f = beta+a*omega/V-delta;
%     a_r = beta-b*omega/V;

    global grip input

    a_r = beta;
    a_f = beta-delta;

    % calculate lateral load transfer (N)
    WT = A_y*cg*W/mean([twf twr]);
    WTF = WT*LLTD;
    WTR = WT*(1-LLTD);

    %Calculate wheel loads with weight transfer
    wfin = wf-WTF;
    wfout = wf+WTF;
    wrin = wr-WTR;
    wrout = wr+WTR;
    
    % Camber = 0 for now
    IA = 0;

    % calculate lateral force from front tires
    F_fin = -MF52_Fy_fcn([a_f wfin IA])*sf_y*cos(delta); % inputs = (rad Newtons rad)
    F_fout = MF52_Fy_fcn([a_f wfout IA])*sf_y*cos(delta);

    % calculate the drag from aero and the front tires (N)
    F_xDrag = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 

    % calculate the grip penalty assuming the rears must overcome that drag
    rscale = 1-(F_xDrag/W/(polyval(grip,V)))^2; % Comes from traction circle

    % now calculate rear tire forces, with said penalty
    F_rin = MF52_Fy_fcn([a_r wrin IA])*sf_y*rscale; 
    F_rout = MF52_Fy_fcn([a_r wrout IA])*sf_y*rscale;

    F_f = F_fin + F_fout;
    F_r = F_rin + F_rout;
    
    % Calculate turning radius from slip angles
    phi_f = pi/2 + a_f - delta;
    phi_r = pi/2 - a_r;

    R_cg_alpha = sqrt((b-L*tan(phi_r)/(tan(phi_f)+tan(phi_r)))^2 + (L*tan(phi_r)*tan(phi_f)/(tan(phi_f)+tan(phi_r)))^2);
    R_cg_v = V^2/A_y;
    
    F(1) = F_f + F_r - A_y*W;
    F(2) = F_f*a - F_r*b;
    F(3) = R_cg_alpha - R_cg_v;

end