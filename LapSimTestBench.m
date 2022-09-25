% Jonathan Vogel
% Clemson Formula SAE
% Tradespace Analysis Project
% 2019 Michigan Dynamic Event Lap Sim

% 
%            _____ _____ ______ _      ______ _____         _______ _____ ____  _   _   _____  _____   _____ _   _    _____ _  _____  __     ______  _    _   _____  _    _ __  __ ____  ______ _    _  _____ _  __
%      /\   / ____/ ____|  ____| |    |  ____|  __ \     /\|__   __|_   _/ __ \| \ | | |_   _|/ ____| |_   _| \ | |  / ____( )/ ____| \ \   / / __ \| |  | | |  __ \| |  | |  \/  |  _ \|  ____| |  | |/ ____| |/ /
%     /  \ | |   | |    | |__  | |    | |__  | |__) |   /  \  | |    | || |  | |  \| |   | | | (___     | | |  \| | | |  __|/| (___    \ \_/ / |  | | |  | | | |  | | |  | | \  / | |_) | |__  | |  | | |    | ' / 
%    / /\ \| |   | |    |  __| | |    |  __| |  _  /   / /\ \ | |    | || |  | | . ` |   | |  \___ \    | | | . ` | | | |_ |  \___ \    \   /| |  | | |  | | | |  | | |  | | |\/| |  _ <|  __| | |  | | |    |  <  
%   / ____ \ |___| |____| |____| |____| |____| | \ \  / ____ \| |   _| || |__| | |\  |  _| |_ ____) |  _| |_| |\  | | |__| |  ____) |    | | | |__| | |__| | | |__| | |__| | |  | | |_) | |    | |__| | |____| . \ 
%  /_/    \_\_____\_____|______|______|______|_|  \_\/_/    \_\_| g |_____\____/|_| \_| |_____|_____/  |_____|_| \_|  \_____| |_____/     |_|  \____/ \____/  |_____/ \____/|_|  |_|____/|_|     \____/ \_____|_|\_\
%                                                                                                                                                                                                                  

clear all
close all
clc

% The purpose of this code is to evaluate the points-scoring capacity of a
% virtual vehicle around the 2019 FSAE Michigan Dynamic Event Tracks

%% Section 0: Name all symbolic variables
% Don't touch this. This is just naming a bunch of variables and making
% them global so that all the other functions can access them
global r_max accel grip deccel lateral cornering gear shift_points...
    top_speed r_min path_boundaries tire_radius shift_time...
    powertrainpackage track_width path_boundaries_ax
%% Section 1: Input Tire Model
% this section is required, everything should be pre-loaded so no need to
% touch any of this, unless you want to change the tire being evaluated.
% The only things you might want to change are the scaling factors at the
% bottom of the section
disp('2019 Michigan Endurance Points Analysis')
disp('Loading Tire Model')

% First we load in the lateral tire force model, which is a Pacejka model
% created by derek:
global FZ0 LFZO LCX LMUX LEX LKX  LHX LVX LCY LMUY LEY LKY LHY LVY ...
       LGAY Ltr LRES LGAZ LXAL LYKA LVYKA LS LSGKP  LSGAL LGYR KY
load('A1654run21_MF52_Fy_GV12.mat')
% then load in coefficients for Magic Formula 5.2 Tire Model:
load('A1654run21_MF52_Fy_12.mat')

% Next you load in the longitudinal tire model, which for now is just a
% CSAPS spline fit to the TTC data
% find your pathname and filename for the tire you want to load in
filename = 'Hoosier_R25B_18.0x7.5-10_FX_12psi.mat';
load(filename)
tire_radius = 9.05/12; %ft
tyreRadius = tire_radius/3.28; % converts to meters

% finally, we have some scaling factors for longitudinal (x) and lateral
% (y) friction. You can use these to tune the lap sim to correlate better 
% to logged data 
sf_x = .6;
sf_y = .47;   
%% Section 2: Input Powertrain Model
% change whatever you want here, this is the 2018 powertrain package iirc
% just keep your units consistent please
disp('Loading Engine Model')
engineSpeed = [6200:100:14100]; % RPM
% torque should be in N-m:
engineTq = [41.57 42.98 44.43 45.65 46.44 47.09 47.52 48.58 49.57 50.41 51.43 51.48 51 49.311 48.94 48.66 49.62 49.60 47.89 47.91 48.09 48.57 49.07 49.31 49.58 49.56 49.84 50.10 50.00 50.00 50.75 51.25 52.01 52.44 52.59 52.73 53.34 53.72 52.11 52.25 51.66 50.5 50.34 50.50 50.50 50.55 50.63 50.17 50.80 49.73 49.35 49.11 48.65 48.28 48.28 47.99 47.68 47.43 47.07 46.67 45.49 45.37 44.67 43.8 43.0 42.3 42.00 41.96 41.70 40.43 39.83 38.60 38.46 37.56 36.34 35.35 33.75 33.54 32.63 31.63];
primaryReduction = 76/36; % Transmission primary reduction (applies to all gears)
gear = [33/12, 32/16, 30/18, 26/18, 30/23, 29/24]; % transmission gear ratios
finalDrive = 40/12; % large sprocket/small sprocket
shiftpoint = 14000; % optimal shiftpoint for most gears [RPM]
drivetrainLosses = .85; % percent of torque that makes it to the rear wheels 
shift_time = .25; % seconds
T_lock = 90; % differential locking torque (0 =  open, 1 = locked)

% Intermediary Calcs/Save your results into the workspace
gearTot = gear(end)*finalDrive*primaryReduction; % Final gear ratio after driven sprocket
VMAX = floor(3.28*shiftpoint/(gearTot/tyreRadius*60/(2*pi))); % Max veloctiy at shiftpoint with unit converstion to ft/s
T_lock = T_lock/100; % Convert to scale factor
powertrainpackage = {engineSpeed engineTq primaryReduction gear finalDrive shiftpoint drivetrainLosses}; % Total powertrain package to be called by supsequent functions
%% Section 3: Vehicle Architecture
disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = 51.5; % Front lateral load transfer distribution (%)
W = 660; % vehicle + driver weight (lbs)
WDF = 50; % front weight distribution (%)
cg = 13.2/12; % center of gravity height (ft)
l = 60.5/12; % wheelbase (ft)
twf = 46/12; % front track width (ft)
twr = 44/12; % rear track width (ft)

% some intermediary calcs you don't have to touch
LLTD = LLTD/100; % Convert to fraction
WDF = WDF/100; % convert to fraction
m = W/32.2; % mass (lbm)
WF = W*WDF; % front weight
WR = W*(1-WDF); % rear weight
a = l*(1-WDF); % front axle to cg
b = l*WDF; % rear axle to cg
tw = twf; % setting larger track width to be used for boundaries in track generation
%% Section 4: Input Suspension Kinematics
disp('Loading Suspension Kinematics')
% this section is actually optional. So if you set everything to zero, you
% can essentially leave this portion out of the analysis. Useful if you are
% only trying to explore some higher level relationships

% Pitch and roll gradients define how much the car's gonna move around
rg_f = 0; % front roll gradient (deg/g)
rg_r = 0; % rear roll gradient (deg/g)
pg = 0; % pitch gradient (deg/g)
WRF = 180; % front and rear ride rates (lbs/in)
WRR = 180; 

% then you can select your camber alignment
IA_staticf = 0; % front static camber angle (deg)
IA_staticr = 0; % rear static camber angle (deg)
IA_compensationf = 10; % front camber compensation (%)
IA_compensationr = 20; % rear camber compensation (%)

% lastly you can select your kingpin axis parameters
casterf = 0; % front caster angle (deg)
KPIf = 0; % front kingpin inclination angle (deg)
casterr = 4.1568; % rear caster angle (deg)
KPIr = 0; % rear kingpin inclination angle (deg)

% intermediary calcs, plz ignore
IA_staticf = deg2rad(IA_staticf); % front static camber angle (deg)
IA_staticr = deg2rad(IA_staticr); % rear static camber angle (deg)
IA_compensationf = IA_compensationf/100; % front camber compensation (%)
IA_compensationr = IA_compensationr/100; % rear camber compensation (%)
casterf = deg2rad(casterf);
KPIf = deg2rad(KPIf);
casterr = deg2rad(casterr);
KPIr = deg2rad(KPIr);
IA_roll_inducedf = asin(2/twf/12); 
IA_roll_inducedr = asin(2/twr/12);
IA_gainf = IA_roll_inducedf*IA_compensationf; % front and rear camber gain per inch of suspension travel (deg/in)
IA_gainr = IA_roll_inducedr*IA_compensationr;
%% Section 5: Input Aero Parameters
disp('Loading Aero Model')
Cl = .0418; %279/418
Cd = .0184; % .0245
CoP = 48; % front downforce distribution (%)

% Intermediary Calculations
CoP = CoP/100;
%% Section 6: Generate GGV Diagram
% this is where the m e a t of the lap sim takes place. The GGV diagram is
% built by finding a maximum cornering, braking, and acceleration capacity
% for any given speed

disp('Generating g-g-V Diagram')

deltar = 0;
deltaf = 0;
velocity = 15:5:130; % range of velocities at which sim will evaluate (ft/s)
radii = [15:10:155]; % range of turn radii at which sim will evaluate (ft)

% First we will evaluate our Acceleration Capacity
g = 1; % g is a gear indicator, and it will start at 1
spcount = 1; % spcount is keeping track of how many gearshifts there are
% shift_points tracks the actual shift point velocities
shift_points(1) = 0; 
disp('     Acceleration Envelope')
for  i = 1:1:length(velocity) % for each velocity
    gp = g; % Current gear = current gear (wow!)
    V = velocity(i); % find velocity
    LF = Cl*V^2; % calculate downforce (lbs)
    % calculate f/r suspension drop from downforce (in)
    dxf = LF*CoP/2/WRF; 
    dxr = LF*(1-CoP)/2/WRR;
    % from rh drop, find camber gain (deg)
    IA_0f = IA_staticf - dxf*IA_gainf;
    IA_0r = IA_staticr - dxr*IA_gainr;
    % find load on each tire (lbs)
    wf = (WF+LF*CoP)/2;
    wr = (WR+LF*(1-CoP))/2;
    
    % now we actually sweep through with acceleration
    Ax = 0; % starting guess of zero g's
    WS = W/2; % weight of one half-car
    pitch = -Ax*pg*pi/180; % pitch angle (rad)
    % recalculate wheel loads due to load transfer (lbs)
    wf = wf-Ax*cg*WS/l; 
    wr = wr+Ax*cg*WS/l;
    % recalculate camber angles due to pitch
    IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;
    IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;
    % select a range of slip ratios (sl) [-]
    sl = [0:.01:.11];
    % evaluate the tractive force capacity from each tire for the range of
    % slip ratios
    for k = 1:length(sl)  
        fxf(k) = fnval([sl(k);-wf;rad2deg(-IA_f)],full_send_x)*sf_x;
        fxr(k) = fnval([sl(k);-wr;rad2deg(-IA_r)],full_send_x)*sf_x;
    end
    % find max force capacity from each tire:
    fxf(find(abs(fxf) > 1000)) = [];
    fxr(find(abs(fxr) > 1000)) = [];
    FXF = max(fxf);
    FXR = max(fxr);
    % Calculate total tire tractive force (lbs)
    FX = abs(2*FXR);
    % calculate total lateral acceleration capacity (g's)
    AX = FX/W;
    AX_diff = AX-Ax;
    while AX_diff>0
        %disp([Ax AX])
        Ax = Ax+.01; % increment acceleration values by .01
        WS = W/2;
        pitch = -Ax*pg*pi/180;
        wf = (WF+LF*CoP)/2;
        wr = (WR+LF*(1-CoP))/2;
        wf = wf-Ax*cg*WS/l/24;
        wr = wr+Ax*cg*WS/l/24;
        IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;% - KPIf*(1-cos(deltaf)) + casterf*sin(deltaf);
        IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;% - KPIr*(1-cos(deltar)) + casterf*sin(deltar);
        FZ_vals = [-250:1:-50];
        sl = [0:.01:.11];
        % evaluate the 
        for k = 1:length(sl)
            fxf(k) = fnval([sl(k);-wf;rad2deg(-IA_f)],full_send_x)*sf_x;
            fxr(k) = fnval([sl(k);-wr;rad2deg(-IA_r)],full_send_x)*sf_x;
        end
        fxf(find(abs(fxf) > 1000)) = [];
        fxr(find(abs(fxr) > 1000)) = [];
        FXF = max(fxf);
        FXR = max(fxr);
        FX = abs(2*FXR);
        AX = FX/W; % THIS IS IN G'S, SEE ABOVE
        AX_diff = AX-Ax;
    end
    A_xr(i) = AX; % little x defines the grip limited maximum accleration
    output = Powertrainlapsim(max(7.5,V/3.28)); % 7.5 reg, 10 launch
    FX = output(1)*.2248;
    FX = FX-Cd*V^2;
    fx(i) = FX/W;
    AX(i) = min(FX/W,A_xr(i));
    output = Powertrainlapsim(V/3.28);
    g = output(2);
    gear(i) = g;
    if g>gp
        spcount = spcount+1;
        shift_points(spcount) = V;
    end
    A_Xr(i) = AX(i); % big X defines the power limited maximum acceleration
end
A_Xr(A_Xr < 0) = 0;

% from these results, you can create the first part of the GGV diagram
% input for the lap sim codes:
% accel is the maximum acceleration capacity as a function of velocity
% (power limited) and grip is the same but (tire limited)
accel = csaps(velocity,A_Xr);clc
grip = csaps(velocity,A_xr);

AYP = 1;

% for turn = 1:1:length(radii)
    % first define your vehicle characteristics:
        a = l*(1-WDF);
        b = l*WDF;
        R = radii(1);
        % update speed and downforce
        V = sqrt(R*32.2*AYP);
        LF = Cl*V^2; 
        % from downforce, update suspension travel (in):
        dxf = LF*CoP/2/WRF; 
        dxr = LF*(1-CoP)/2/WRR; 
        % from suspension heave, update static camber (rad):
        IA_0f = IA_staticf - dxf*IA_gainf; 
        IA_0r = IA_staticr - dxr*IA_gainr; 
        % update load on each axle (lbs)
        wf = (WF+LF*CoP)/2;
        wr = (WR+LF*(1-CoP))/2;
        % guess ackermann steer angle as starting steer angle
        delta = l/R;
        % assume vehicle sideslip starts at 0 (rad)
        beta = deg2rad(0);
        A_y = V^2/R;
        % calculate lateral load transfer (lbs)
        WT = A_y*cg*W/mean([twf twr])/32.2/12;
        % split f/r using LLTD
        WTF = WT*LLTD;
        WTR = WT*(1-LLTD);
        % calculate f/r roll (rad)
        phif = A_y*rg_f*pi/180/32.2;
        phir = A_y*rg_r*pi/180/32.2;
        % update individual wheel loads 
        wfin = wf-WTF;
        wfout = wf+WTF;
        wrin = wr-WTR;
        wrout = wr+WTR;
        % update individual wheel camber (from roll, then from steer
        % effects)
        IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
        IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
        IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir;
        % calculate yaw rate
        r = A_y/V;
        % from yaw, sideslip and steer you can get slip angles
        a_f = beta+a*r/V-delta;
        a_r = beta-b*r/V;
        % with slip angles, load and camber, calculate lateral force at
        % the front
        F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
        F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
        % before you calculate the rears, you ned to see what the diff is
        % doing
        % calculate the drag from aero and the front tires
        F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
        % calculate the grip penalty assuming the rears must overcome that
        % drag
        rscale = 1-(F_x/W/fnval(grip,V))^2; % WHY THE FUCK IS IT SQUARED %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % now calculate rear tire forces, with said penalty
        F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
        F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
        % sum of forces and moments
        F_y = F_fin+F_fout+F_rin+F_rout;
        M_z_diff = F_x*T_lock*twr/2; % incl the differential contribution %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
        % calculate resultant lateral acceleration
        AY = F_y/(W/32.2);
        % compare to the initial guess
        diff_AY = A_y-AY;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        x0 = [delta, beta, AYP];
        f = @(x)vogel(x,a,b,Cd,IA_gainf,IA_gainr,twf,KPIf,cg,W,twr,LLTD,rg_r,rg_f,casterf,KPIr,deltar,sf_y,T_lock,V,R,wf,wr,WTR,IA_0f,IA_0r,A);
        initialRun = f(x0)
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        nonlcon = [];
        lb = [-15 -15 0];
        ub = [15 15 3];
        [x,fval,exitflag] = fmincon(f,x0,A,b,Aeq,beq,lb,ub);
%         opts_ps = optimoptions('paretosearch','Display','off','PlotFcn','psplotparetof');
%         rng default % For reproducibility
%         [x_ps1,fval_ps1,~,psoutput1] = paretosearch(f,3,A,b,Aeq,Beq,lb,ub,nonlcon,opts_ps);
        
% 
%         a = l*(1-WDF);
%         b = l*WDF;
%         R = radii(1);
%         % update speed and downforce
%         V = sqrt(R*32.2*AYP);
%         LF = Cl*V^2; 
%         % from downforce, update suspension travel (in):
%         dxf = LF*CoP/2/WRF; 
%         dxr = LF*(1-CoP)/2/WRR; 
%         % from suspension heave, update static camber (rad):
%         IA_0f = IA_staticf - dxf*IA_gainf; 
%         IA_0r = IA_staticr - dxr*IA_gainr; 
%         % update load on each axle (lbs)
%         wf = (WF+LF*CoP)/2;
%         wr = (WR+LF*(1-CoP))/2;
%         % guess ackermann steer angle as starting steer angle
%         delta = l/R;
%         ddelta = delta*.01;
%         % assume vehicle sideslip starts at 0 (rad)
%         beta = deg2rad(0);
%         A_y = V^2/R;
%         % calculate lateral load transfer (lbs)
%         WT = A_y*cg*W/mean([twf twr])/32.2/12;
%         % split f/r using LLTD
%         WTF = WT*LLTD;
%         WTR = WT*(1-LLTD);
%         % calculate f/r roll (rad)
%         phif = A_y*rg_f*pi/180/32.2;
%         phir = A_y*rg_r*pi/180/32.2;
%         % update individual wheel loads 
%         wfin = wf-WTF;
%         wfout = wf+WTF;
%         wrin = wr-WTR;
%         wrout = wr+WTR;
%         % update individual wheel camber (from roll, then from steer
%         % effects)
%         IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
%         IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
%         IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
%         IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir;
%         % calculate yaw rate
%         r = A_y/V;
%         % from yaw, sideslip and steer you can get slip angles
%         a_f = beta+a*r/V-delta;
%         a_r = beta-b*r/V;
%         % with slip angles, load and camber, calculate lateral force at
%         % the front
%         F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
%         F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
%         % before you calculate the rears, you ned to see what the diff is
%         % doing
%         % calculate the drag from aero and the front tires
%         F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
%         % calculate the grip penalty assuming the rears must overcome that
%         % drag
%         rscale = 1-(F_x/W/fnval(grip,V))^2; % WHY THE FUCK IS IT SQUARED %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         % now calculate rear tire forces, with said penalty
%         F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
%         F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
%         % sum of forces and moments
%         F_y = F_fin+F_fout+F_rin+F_rout;
%         M_z_diff = F_x*T_lock*twr/2; % incl the differential contribution %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%         M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
%         % calculate resultant lateral acceleration
%         AY = F_y/(W/32.2);
% 
% %         B = rad2deg(beta);
% %         af = rad2deg(a_f);
% %         ar = rad2deg(a_r);
% %         steer = rad2deg(delta);
% %         UG = rad2deg(delta-l/R)*32.2/AY;
% %         Ugradient(turn) = UG;
% %         %F_lat = fnval([rad2deg(a_f);-wf;0],full_send_y)*.45*cos(delta);
% %         %F_drag = fnval([rad2deg(a_f);-wf;0],full_send_y)*.45*sin(delta);
% %         skid = 2*pi*R/V;
% %         steering(turn) = steer;
% %         speed(turn) = V;
% %         lateralg(turn) = AY/32.2;
% 
%         B = rad2deg(beta);
%         af = rad2deg(a_f);
%         ar = rad2deg(a_r);
%         steer = rad2deg(delta);
%         UG = rad2deg(delta-l/R)*32.2/AY;
%         Ugradient(1) = UG;
%         %F_lat = fnval([rad2deg(a_f);-wf;0],full_send_y)*.45*cos(delta);
%         %F_drag = fnval([rad2deg(a_f);-wf;0],full_send_y)*.45*sin(delta);
%         skid = 2*pi*R/V;
%         steering(1) = steer;
%         speed(1) = V;
%         lateralg(1) = AY/32.2;
% % end
