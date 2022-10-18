function F = latVehState(delta, beta, A_y, V,)
    
%     % Calculate turn radius
%     R = V^2/A_y;
% 
%     %calculate yaw rate
%     omega = V/R;
% 
%     %from yaw, sideslip and steer you can get slip angles (rad)
%     a_f = beta+a*omega/V-delta;
%     a_r = beta-b*omega/V;

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
    
    % calculate lateral force from front tires
    F_fin = -MF52_Fy_fcn([a_f wfin -IA_f_in])*sf_y*cos(delta); % inputs = (rad Newtons rad)
    F_fout = MF52_Fy_fcn([a_f wfout -IA_f_out])*sf_y*cos(delta);

    % calculate the drag from aero and the front tires (N)
    F_xDrag = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 

    % calculate the grip penalty assuming the rears must overcome that drag
    rscale = 1-(F_xDrag/W/(polyval(grip,V)))^2; % Comes from traction circle

    % now calculate rear tire forces, with said penalty
    F_rin = -MF52_Fy_fcn([-a_r wrin -IA_r_in])*sf_y*rscale; 
    F_rout = MF52_Fy_fcn([a_r wrout -IA_r_out])*sf_y*rscale;

    F_f = F_fin + F_fout;
    F_r = F_rin + F_rout;
    
    F(1) = F_f + F_r - A_y*W;
    F(2) = F_f*a - F_r*b;

end