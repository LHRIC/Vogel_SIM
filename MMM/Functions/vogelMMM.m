function [output] =vogelMMM(x,a,b,Cd,IA_gainf,IA_gainr,twf,KPIf,cg,W,twr,LLTD,rg_r,rg_f, ...
    casterf,KPIr,deltar,sf_y,T_lock,R,wf,wr,IA_0f,IA_0r,delta,beta,input)

global grip input
        % Convert SA from deg2rad
        beta  = deg2rad(beta);
        delta = deg2rad(delta);
        %
        AYP = x(1);
        A_y = AYP; % (g's)
        % calculate lateral load transfer (N)
        WT = A_y*cg*W/mean([twf twr]);
        % split f/r using LLTD (N)
        WTF = WT*LLTD;
        WTR = WT*(1-LLTD);
        % calculate f/r roll (rad)
        phif = A_y*rg_f;
        phir = A_y*rg_r;
        % update individual wheel loads 
        wfin = wf-WTF;
        wfout = wf+WTF;
        wrin = wr-WTR;
        wrout = wr+WTR;
        % update individual wheel camber (rad) (from roll, then from steer
        % effects)
        IA_f_in = -twf*sin(phif)/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
        IA_f_out = -twf*sin(phif)/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
        IA_r_in = -twr*sin(phir)/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
        % Convert Tire Frame Slip Angles to Velocity Frame
        a_r = beta;
        a_f = delta+beta;
        % with slip angles, load and camber, calculate lateral force at
        % the front
        F_fin = -MF52_Fy_fcn([-a_f wfin -IA_f_in])*sf_y*cos(delta); % inputs = (rad Newtons rad)
        F_fout = MF52_Fy_fcn([a_f wfout -IA_f_out])*sf_y*cos(delta);
        % calculate the drag from aero and the front tires (N)
        %F_xDrag = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); % Currently Unused
        % calculate the grip penalty assuming the rears must overcome that
        % drag
        rscale = 1; % (set to 1 to consider no conteracting acccel) 1-(F_xDrag/W/(polyval(grip,V)))^2; % Comes from traction circle
        % now calculate rear tire forces, with said penalty
        F_rin = -MF52_Fy_fcn([-a_r wrin -IA_r_in])*sf_y*rscale; 
        F_rout = MF52_Fy_fcn([a_r wrout -IA_r_out])*sf_y*rscale;
        % sum of forces and moments
        F_y = F_fin+F_fout+F_rin+F_rout;
        M_z_diff = 0; % F_xDrag*T_lock*twr/2; (Ignoring Diff Effects)       
        M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
        % calculate resultant lateral acceleration
        AY = F_y/(W);
        diff_AY = A_y-AY;
        if input == 1
        output = [diff_AY];
        else
        output = [A_y M_z a_f a_r];
        end

end