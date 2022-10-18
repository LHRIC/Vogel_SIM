function [output] =vogelMMM(x,a,b,Cd,IA_gainf,IA_gainr,twf,KPIf,cg,W,twr,LLTD,rg_r,rg_f, ...
    casterf,KPIr,deltar,sf_y,T_lock,R,wf,wr,IA_0f,IA_0r,vel,steered_angle,body_angle,input)

global grip input
        % Convert SA from deg2rad
        body_angle  = deg2rad(body_angle);
        steered_angle = deg2rad(steered_angle);
        %
        AYP = x(1);
        yaw_rate = x(2);
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
        IA_f_in = -twf*sin(phif)/2*IA_gainf - IA_0f - KPIf*(1-cos(steered_angle)) - casterf*sin(steered_angle) +phif;
        IA_f_out = -twf*sin(phif)/2*IA_gainf + IA_0f + KPIf*(1-cos(steered_angle)) - casterf*sin(steered_angle) + phif;
        IA_r_in = -twr*sin(phir)/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 

        % Convert All Tire Angles to Velocity Frame (Steering slip
        % contribution)
        % *Aside* could add toe angle here  and below later. Would create
        % left and right tire angle then. Could also add ackermann here
        a_r = body_angle;
        a_f = steered_angle+body_angle;
        % Find yaw rate slip angle contribution(Left outside)(Body Frame)
        B_f_in  = atan((vel*sin(body_angle)-yaw_rate*a)/(vel*cos(body_angle)+yaw_rate*(twf/2)));
        B_f_out = atan((vel*sin(body_angle)-yaw_rate*a)/(vel*cos(body_angle)+yaw_rate*(-(twf/2))));
        B_r_in  = atan((vel*sin(body_angle)-yaw_rate*(-b))/(vel*cos(body_angle)+yaw_rate*(twr/2)));
        B_r_out = atan((vel*sin(body_angle)-yaw_rate*(-b))/(vel*cos(body_angle)+yaw_rate*(-(twr/2))));
        % Find slip angle with yaw rate correction (Velocity Frame) 
        % (SA = diff between direction of travel tire heading)
        a_f_in = a_f+B_f_in;
        a_f_out = a_f+B_f_out;
        a_r_in = a_r+B_r_in;
        a_r_out = a_r+B_r_out;

        % with slip angles, load and camber, calculate lateral force at
        % the front
        F_fin = -MF52_Fy_fcn([-a_f_in wfin -IA_f_in])*sf_y*cos(steered_angle); % inputs = (rad Newtons rad)
        F_fout = MF52_Fy_fcn([a_f_out wfout -IA_f_out])*sf_y*cos(steered_angle);
        % calculate the drag from aero and the front tires (N)
        %F_xDrag = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); % Currently Unused
        % calculate the grip penalty assuming the rears must overcome that
        % drag
        rscale = 1; % (set to 1 to consider no conteracting acccel for MMM) 1-(F_xDrag/W/(polyval(grip,V)))^2; % Comes from traction circle
        % now calculate rear tire forces, with said penalty
        F_rin = -MF52_Fy_fcn([-a_r_in wrin -IA_r_in])*sf_y*rscale; 
        F_rout = MF52_Fy_fcn([a_r_out wrout -IA_r_out])*sf_y*rscale;
        % sum of forces and moments
        F_y = F_fin+F_fout+F_rin+F_rout;
        M_z_diff = 0; % F_xDrag*T_lock*twr/2; (Ignoring Diff Effects)       
        M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
        % calculate resultant lateral acceleration
        AY = F_y/(W);
        % calculate yaw rate
        radius = vel^2/AY; % first calculate turn radius
        yaw_rate_guess = vel/radius;% then calculate yaw rate
        % Differentals for paratmers to be optimized
        diff_AY = A_y-AY;
        diff_yaw_rate = yaw_rate-yaw_rate_guess;
        %Output
        if input == 1
        output = [diff_AY diff_yaw_rate];
        else
        output = [A_y M_z a_f a_r yaw_rate yaw_rate];
        end

end