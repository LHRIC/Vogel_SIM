function [output] =vogelMMM(x,a,b,Cd,IA_gainf,IA_gainr,twf,KPIf,cg,W,twr,LLTD,rg_r,rg_f, ...
    casterf,KPIr,deltar,sf_y,T_lock,R,wf,wr,IA_0f,IA_0r,vel,steered_angle,body_angle,input)

global grip input
        % Convert SA from deg2rad
        body_angle  = deg2rad(body_angle);
        steered_angle = deg2rad(steered_angle);
        %
        AY_NTB = x(1);
        yaw_rate = x(2);
        AX_NTB = x(3);
        AY_IMF = AY_NTB*cos(body_angle) + AX_NTB*sin(body_angle); % (g's)
        AX_IMF = AX_NTB*cos(body_angle) - AY_NTB*sin(body_angle); % (g's)


        % [fin fout rin rout]
        % [fr  fl   rr  rl  ]
        % Calculate Weight Transfer
        LatWT  = AY_IMF*cg*W/mean([twf twr]); % calculate lateral load transfer (N)
        LongWT = AX_IMF*W*cg/(a+b);
        % split f/r using LLTD (N)
        LatWTF = LatWT*LLTD;
        LatWTR = LatWT*(1-LLTD);
        % update individual wheel loads 
        wfin  = wf-LatWTF-LongWT;
        wfout = wf+LatWTF-LongWT;
        wrin  = wr-LatWTR+LongWT;
        wrout = wr+LatWTR+LongWT;

        % Calculate suspension kinematics changes *** Unused here ***
        phif = AY_IMF*rg_f;% calculate f/r roll (rad)
        phir = AY_IMF*rg_r;
        % update individual wheel camber (rad) (from roll, then from steer
        % effects)
        IA_f_in = -twf*sin(phif)/2*IA_gainf - IA_0f - KPIf*(1-cos(steered_angle)) - casterf*sin(steered_angle) +phif;
        IA_f_out = -twf*sin(phif)/2*IA_gainf + IA_0f + KPIf*(1-cos(steered_angle)) - casterf*sin(steered_angle) + phif;
        IA_r_in = -twr*sin(phir)/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 

        % Convert All Tire Angles to NTB Frame (Steering slip
        % contribution)
        % *Aside* could add toe angle here  and below later. Would create
        % left and right tire angle then. Could also add ackermann here
        % (NTB Frame)
        a_r = body_angle;
        a_f = steered_angle+body_angle;
        % Find yaw rate slip angle contribution (Left outside) (NTB Frame)
        B_f_in  = atan((vel*sin(-body_angle)-yaw_rate*a)   /(vel*cos(-body_angle)+yaw_rate*(twf/2)));
        B_f_out = atan((vel*sin(-body_angle)-yaw_rate*a)   /(vel*cos(-body_angle)+yaw_rate*((-twf/2))));
        B_r_in  = atan((vel*sin(-body_angle)-yaw_rate*(-b))/(vel*cos(-body_angle)+yaw_rate*(twr/2)));
        B_r_out = atan((vel*sin(-body_angle)-yaw_rate*(-b))/(vel*cos(-body_angle)+yaw_rate*((-twr/2))));
        % Find slip angle with yaw rate correction (NTB to Tire Frame) 
        % (SA = diff between direction of travel tire heading)
        a_f_in  = a_f-B_f_in;
        a_f_out = a_f-B_f_out;
        a_r_in  = a_r-B_r_in;
        a_r_out = a_r-B_r_out;

        % CALCULATE TIRE FORCES
        % with slip angles, load and camber, calculate lateral force at
        % the front (IMF FRAME)
        F_fin  =-MF52_Fy_fcn([-a_f_in wfin -IA_f_in])*sf_y*cos(steered_angle); % inputs = (rad Newtons rad)
        F_fout = MF52_Fy_fcn([a_f_out wfout -IA_f_out])*sf_y*cos(steered_angle);
        % calculate the drag from aero and the front tires (N)
        % calculate the grip penalty assuming the rears must overcome that
        % drag
        rscale = 1; % (set to 1 to consider no conteracting acccel for MMM) 1-(F_xDrag/W/(polyval(grip,V)))^2; % Comes from traction circle
        % now calculate rear tire forces (Tire Frame *For Rear this is also IMF*)
        F_rin  =-MF52_Fy_fcn([-a_r_in wrin -IA_r_in])*sf_y*rscale;
        F_rout = MF52_Fy_fcn([a_r_out wrout -IA_r_out])*sf_y*rscale;
        % Calculate long accel from all tires and aero force (N) (IMF Frame) *For Later calculate aero force in relation to velocity components*
        F_x_IMF = -Cd*vel^2 + (F_fin+F_fout)*tan(steered_angle);
        % sum of forces and moments
        F_y_IMF = F_fin+F_fout+F_rin+F_rout; % IMF Frame
        M_z_diff = 0; % F_xDrag*T_lock*twr/2; (Ignoring Diff Effects) (Should look into this later)     
        M_z = F_fout*tan(steered_angle)*twf/2 - F_fin*tan(steered_angle)*twf/2 + (F_fin+F_fout)*a - (F_rin+F_rout)*b - M_z_diff; % IMF (NEED TO VERIFY THIS. LONG COMPONENT OF M_z May be lost in Fy to IMF conversion. I dont keep both X and Y components)
        % calculate resultant accelerations (IMF Frame)
        AY_IMF_Guess = F_y_IMF/W;
        AX_IMF_Guess = F_x_IMF/W;
        % Calculate Accelerations in (NTB Frame)
        AY_NTB_Guess = AY_IMF_Guess*cos(body_angle) - AX_IMF_Guess*sin(body_angle);
        AX_NTB_Guess = AX_IMF_Guess*cos(body_angle) + AY_IMF_Guess*sin(body_angle);
        % calculate yaw rate
        radius = vel^2/AY_NTB_Guess; % first calculate turn radius
        yaw_rate_guess = vel/radius;% then calculate yaw rate

        % Differentals for paratmers to be optimized
        diff_AY = AY_NTB-AY_NTB_Guess;
        diff_yaw_rate = yaw_rate-yaw_rate_guess;
        diff_AX = AX_NTB-AX_NTB_Guess;
        %Output
        if input == 1
        output = [diff_AY diff_yaw_rate diff_AX];
        else
        output = [AY_NTB M_z a_f a_r yaw_rate AX_NTB];
        end

end