function [output]=LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, casterr, KPIr, Cl, Cd, CoP, showPlots)
%% Section 0: Name all symbolic variables
% Don't touch this. This is just naming a bunch of variables and making
% them global so that all the other functions can access them
global r_max accel grip deccel lateral cornering gear shift_points...
    top_speed r_min path_boundaries tire_radius shift_time...
    powertrainpackage track_width path_boundaries_ax OptimParameterSet
%% Section 1: Input Tire Model
% this section is required, everything should be pre-loaded so no need to
% touch any of this, unless you want to change the tire being evaluated.
% The only things you might want to change are the scaling factors at the
% bottom of the section
%disp('2019 Michigan Endurance Points Analysis')
%disp('Loading Tire Model')



global FZ0
FZ0 = 800; %Tire nominal load (N)
load("Tire Modeling/Lateral_Tire-Model_Optim_Params.mat")
   
% Next you load in the longitudinal tire model
% find your pathname and filename for the tire you want to load in
load("Tire Modeling/12.mat") 
tire_radius = .2032; % (meters)


% finally, we have some scaling factors for longitudinal (x) and lateral
% (y) friction. You can use these to tune the lap sim to correlate better 
% to logged data 
sf_x = .6;
sf_y = .6;   
%% Section 2: Input Powertrain Model
% change whatever you want here, this is the 2018 powertrain package iirc
% just keep your units consistent please

disp('Loading Engine Model')

engineSpeed = 6200:100:14100; % RPM

% torque should be in N-m:
engineTq = [41.57 42.98 44.43 45.65 46.44 47.09 47.52 48.58 49.57 50.41 51.43 51.48 51 49.311 48.94 48.66 49.62 49.60 47.89 47.91 48.09 48.57 49.07 49.31 49.58 49.56 49.84 50.10 50.00 50.00 50.75 51.25 52.01 52.44 52.59 52.73 53.34 53.72 52.11 52.25 51.66 50.5 50.34 50.50 50.50 50.55 50.63 50.17 50.80 49.73 49.35 49.11 48.65 48.28 48.28 47.99 47.68 47.43 47.07 46.67 45.49 45.37 44.67 43.8 43.0 42.3 42.00 41.96 41.70 40.43 39.83 38.60 38.46 37.56 36.34 35.35 33.75 33.54 32.63 31.63];
primaryReduction = 76/36; % Transmission primary reduction (applies to all gears)
gear = [33/12, 32/16, 30/18, 26/18, 30/23, 29/24]; % transmission gear ratios
finalDrive = 37/11; % large sprocket/small sprocket
shiftpoint = 14000; % optimal shiftpoint for most gears [RPM]
drivetrainLosses = .85; % percent of torque that makes it to the rear wheels 
shift_time = .25; % seconds
T_lock = 0; % differential locking torque (0 =  open, 1 = locked)

% Intermediary Calcs/Save your results into the workspace
gearTot = gear(end)*finalDrive*primaryReduction; % Final gear ratio after driven sprocket
VMAX = shiftpoint/(gearTot/tire_radius*60/(2*pi)); % Max veloctiy at shiftpoint
powertrainpackage = {engineSpeed engineTq primaryReduction gear finalDrive shiftpoint drivetrainLosses}; % Total powertrain package to be called by supsequent functions
%% Section 3: Vehicle Architecture

% some intermediary calcs you don't have to touch
m = W/9.81; % mass (Kg)
WF = W*WDF; % front weight
WR = W*(1-WDF); % rear weight
a = L*(1-WDF); % front axle to cg
b = L*WDF; % rear axle to cg
tw = twf; % setting larger track width to be used for boundaries in track generation
%% Section 4: Input Suspension Kinematics
%disp('Loading Suspension Kinematics')
% this section is actually optional. So if you set everything to zero, you
% can essentially leave this portion out of the analysis. Useful if you are
% only trying to explore some higher level relationships

% intermediary calcs, plz ignore
IA_compensationf = IA_compensationf/100; % front camber compensation (%)
IA_compensationr = IA_compensationr/100; % rear camber compensation (%)
casterf = deg2rad(casterf);
KPIf = deg2rad(KPIf);
casterr = deg2rad(casterr);
KPIr = deg2rad(KPIr);
IA_roll_inducedf = real(asin(2/twf));
IA_roll_inducedr = real(asin(2/twr));
IA_gainf = IA_roll_inducedf*IA_compensationf; % front and rear camber gain per meter of suspension travel (deg/m)
IA_gainr = IA_roll_inducedr*IA_compensationr;
%% Section 6: Generate GGV Diagram
% this is where the m e a t of the lap sim takes place. The GGV diagram is
% built by finding a maximum cornering, braking, and acceleration capacity
% for any given speed

disp('Generating g-g-V Diagram')
deltar = 0;
deltaf = 0;
velocity = 4.5:1.5:22.5; % range of velocities at which sim will evaluate (m/s)
radii = 3.5:1.5:36; % range of turn radii at which sim will evaluate (m)
% First we will evaluate our Acceleration Capacity
g = 1; % g is a gear indicator, and it will start at 1
spcount = 1; % spcount is keeping track of how many gearshifts there are
% shift_points tracks the actual shift point velocities
shift_points(1) = 0; 
disp('Longitudinal Acceleration Envelope')
A_xr = zeros(1,length(velocity));
fx = zeros(1,length(velocity));
A_Xr = zeros(1,length(velocity));
for  i = 1:1:length(velocity) % for each velocity
    gp = g; % Current gear = current gear (wow!)
    V = velocity(i); % find velocity
    LF = Cl*V^2; % calculate downforce (N)
    % calculate f/r suspension drop from downforce (m)
    dxr = LF*(1-CoP)/2/WRR;
    % from rh drop, find camber gain (rad)
    IA_0r = IA_staticr - dxr*IA_gainr;
    Ax = 0;
    AX_diff = 1;
    while AX_diff>0
        %disp([Ax AX])
        Ax = Ax+.01; % increment acceleration values by .01
        pitch = -Ax*pg;
        % find load on each tire (N)
        wr = (WR+LF*(1-CoP))/2;
%         wr = wr+Ax*cg*WS/L/24; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        wr = wr+Ax*cg*W/L/2; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        IA_r = L*sin(pitch)/2*IA_gainr + IA_0r;% - KPIr*(1-cos(deltar)) + casterf*sin(deltar);
        sl = 0:.01:1;
        fxr = zeros(1,length(sl));
        for k = 1:length(sl)
              fxr(k) = MF52_Fx_fcn(lCoeff,[-wr -IA_r],sl(k))*sf_x;
        end
%         plot(sl,fxr)
%         hold on
        FXR = max(fxr);
        FX = abs(2*FXR);
        AX = FX/W; % THIS IS IN G'S, SEE ABOVE
        AX_diff = AX-Ax;
    end
    A_xr(i) = AX; % little x defines the grip limited maximum accleration
    output = Powertrainlapsim(max(7.5,V)); % 7.5 reg, 10 launch
    FX = output(1); % Newtons
    FX = FX-Cd*V^2;
    fx(i) = FX/W;
    AX(i) = min(FX/W,A_xr(i));
    output = Powertrainlapsim(V);
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
accel = csaps(velocity,A_Xr);
grip = polyfit(velocity,A_xr,3);



% range = linspace(4.5,30);
% hold off
% figure
% hold on
% plot(velocity,A_xr,'o',velocity,A_Xr,'o',range,polyval(grip,range))
% hold on
% % fnplt(gripspline)
% grid on
% grid minor
% hold on
% fnplt(accel)
% grid on
% grid minor

AYP = .5;
disp('Lateral Acceleration Envelope')

% lateralg = zeros(1,length(radii));
% for turn = 1:1:length(radii)
%     % first define your vehicle characteristics:
%         a = L*(1-WDF);
%         b = L*WDF;
%         R = radii(turn); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         % update speed and downforce
%         V = sqrt(R*9.81*AYP);
%         LF = Cl*V^2; 
%         % from downforce, update suspension travel (m):
%         dxf = LF*CoP/2/WRF; 
%         dxr = LF*(1-CoP)/2/WRR; 
%         % from suspension heave, update static camber (rad):
%         IA_0f = IA_staticf - dxf*IA_gainf; 
%         IA_0r = IA_staticr - dxr*IA_gainr; 
%         % update load on each wheel (N)
%         wf = (WF+LF*CoP)/2;
%         wr = (WR+LF*(1-CoP))/2;
%         % guess ackermann steer angle as starting steer angle
%         delta = L/R;
%         % assume vehicle sideslip starts at 0 (rad)
%         beta = 0;
%         % Minimizing the objective function of M_z a_f-12degrees and a_y-AY 
%         x0 = [delta, beta, AYP];  % initial values
%         fun = @(x)vogel(x,a,b,Cd,IA_gainf,IA_gainr,twf,KPIf,cg,W,twr,LLTD,rg_r, ...
%             rg_f,casterf,KPIr,deltar,sf_y,T_lock,R,wf,wr,IA_0f,IA_0r);
%         fun(x0);
%         % minimizing function
%         lb = [0 -.2 .5];
%         ub = [.5 .2 2];
%         opts = optimoptions("lsqnonlin",MaxFunctionEvaluations=1000000,MaxIterations=1000000,FunctionTolerance=1e-16,Display="none");
%         x = lsqnonlin(fun, x0,lb,ub,opts);
%         % output from minimizing
%         delta = x(1);
%         beta = x(2);
%         AYP = x(3);
%         deltaTest(turn) = rad2deg(delta);
%         betaTest(turn) = rad2deg(beta);
%         AYPTest(turn) = AYP;
%         % recalculate values with optimized delta, beta, and AYP
%         % update speed and downforce
%         V = sqrt(R*9.81*AYP);
%         LF = Cl*V^2; 
%         % from downforce, update suspension travel (m):
%         dxf = LF*CoP/2/WRF; 
%         dxr = LF*(1-CoP)/2/WRR; 
%         % from suspension heave, update static camber (rad):
%         IA_0f = IA_staticf - dxf*IA_gainf; 
%         IA_0r = IA_staticr - dxr*IA_gainr; 
%         % update load on each axle (N)
%         wf = (WF+LF*CoP)/2;
%         wr = (WR+LF*(1-CoP))/2;
%         % assume vehicle sideslip starts at 0 (rad)
%         A_y = V^2/R/9.81; % (g's)
%         % calculate lateral load transfer (lbs)
%         WT = A_y*cg*W/mean([twf twr]);
%         % split f/r using LLTD
%         WTF = WT*LLTD;
%         WTR = WT*(1-LLTD);
%         % calculate f/r roll (rad)
%         phif = A_y*rg_f;
%         phir = A_y*rg_r;
%         % update individual wheel loads 
%         wfin = wf-WTF;
%         wfout = wf+WTF;
%         wrin = wr-WTR;
%         wrout = wr+WTR;
%         wfin1(turn) = wf-WTF;
%         wfout1(turn) = wf+WTF;
%         wrin1(turn) = wr-WTR;
%         wrout1(turn) = wr+WTR;
%         % update individual wheel camber (from roll, then from steer
%         % effects)
%         IA_f_in = -twf*sin(phif)/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
%         IA_f_out = -twf*sin(phif)/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
%         IA_r_in = -twr*sin(phir)/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
%         IA_r_out = -twr*sin(phir)/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir;
%         % calculate yaw rate
%         r = A_y*9.81/V;
%         % from yaw, sideslip and steer you can get slip angles
%         a_f = beta+a*r/V-delta;
%         a_r = beta-b*r/V;
%         % with slip angles, load and camber, calculate lateral force at
%         % the front
%         F_fin = -MF52_Fy_fcn([-a_f wfin -IA_f_in])*sf_y*cos(delta); % inputs = (rad Newtons rad)
%         F_fout = MF52_Fy_fcn([a_f wfout -IA_f_out])*sf_y*cos(delta);
%         % before you calculate the rears, you ned to see what the diff is
%         % doing
%         % calculate the drag from aero and the front tires
%         F_xDrag = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta);
%         f_xplt(turn) = (F_fin+F_fout)*sin(delta)/cos(delta)/400;
%         % calculate the grip penalty assuming the rears must overcome that
%         % drag
%         rscale = 1-(F_xDrag/W/polyval(grip,V))^2; % Comes from traction circle
%         % now calculate rear tire forces, with said penalty
%         F_rin = -MF52_Fy_fcn([-a_r wrin -IA_r_in])*sf_y*rscale; 
%         F_rout = MF52_Fy_fcn([a_r wrout -IA_r_out])*sf_y*rscale;
%         F_fin1(turn) = F_fin;
%         F_fout1(turn) = F_fout;
%         F_rin1(turn) = F_rin;
%         F_rout1(turn) = F_rout;
%         % sum of forces and moments
%         F_y = F_fin+F_fout+F_rin+F_rout;
%         M_z_diff = F_xDrag*T_lock*twr/2;  
%         % calculate resultant lateral acceleration
%         lateralg(turn) = F_y/W;  
% %         M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;       
% %         B = rad2deg(beta);
% %         af = rad2deg(a_f);
% %         ar = rad2deg(a_r);
%         steer = rad2deg(delta);
%         steerTest(turn) = steer;
% %         UG = rad2deg(delta-L/R)*9.81/AY;
% %         Ugradient(1) = UG;
% %         skid = 2*pi*R/V;
% %         steering(turn) = steer;
% %         speed(turn) = V;
% 
% end

load('lateralg.mat')
disp("////////////////////////////////////WARNING////////////////////" + ...
    "LOADING PRECALCULATED LATERALG")

% lateralg 
% figure
% plot(radii,F_fin1,'m',radii,F_fout1,'c',radii, F_rin1,'b',radii,F_rout1,'g')
% legend('Front Inner', 'Front outer', 'Rear Inner', 'Rear Outer')
% grid on
% hold off
% figure
% plot(radii,wfin1,'m',radii,wfout1,'c',radii, wrin1,'b',radii,wrout1,'g')
% legend('Front Inner', 'Front outer', 'Rear Inner', 'Rear Outer')
% grid on
% hold off
% velocity_y = lateralg.*9.81.*radii;
% velocity_y = sqrt(velocity_y);
% range = linspace(4.5,velocity_y(end));
% hold on
% lateral = polyfit(velocity_y,lateralg,3);
% figure
% plot(velocity_y,f_xplt,'o')
% hold on
% plot(velocity_y,lateralg,'o',range,polyval(lateral,range))
% grid on 
% grid minor
% hold off


% Braking Performance
velocity = 4.5:1.5:22.5; % range of velocities at which sim will evaluate (m/s)
disp('     Braking Envelope')
% the braking sim works exactly the same as acceleration, except now all 4
% tires are contributing to the total braking capacity
A_X = zeros(1,length(velocity));
for  i = 1:1:length(velocity)
    V = velocity(i);
    LF = Cl*V^2;
    dxf = LF*CoP/2/WRF;
    dxr = LF*(1-CoP)/2/WRR;
    IA_0f = IA_staticf - dxf*IA_gainf;
    IA_0r = IA_staticr - dxr*IA_gainr;
    wf = (WF+LF*CoP)/2;
    wr = (WR+LF*(1-CoP))/2;
    Ax = 1;
    pitch = Ax*pg;
    wf = wf+Ax*cg*W/L/2;
    wr = wr-Ax*cg*W/L/2;
    IA_f = -L*12*sin(pitch)/2*IA_gainf + IA_0f;% - KPIf*(1-cos(deltaf)) + casterf*sin(deltaf);
    IA_r = L*12*sin(pitch)/2*IA_gainr + IA_0r;% - KPIr*(1-cos(deltar)) + casterf*sin(deltar);
    sl = -.23:.01:0;
    for k = 1:length(sl)
          fxf(k) = MF52_Fx_fcn(lCoeff,[-wf -IA_f],sl(k))*sf_x;
          fxr(k) = MF52_Fx_fcn(lCoeff,[-wr -IA_r],sl(k))*sf_x;
    end
    FXF = min(fxf);
    FXR = min(fxr);
    FX = abs(2*FXF+2*FXR);
    AX = FX/W;
    AX_diff = AX-Ax;
    while AX_diff>0
        Ax = Ax+.01;
        pitch = Ax*pg;
        wf = (WF+LF*CoP)/2;
        wr = (WR+LF*(1-CoP))/2;
        wf = wf+Ax*cg*W/L/2;
        wr = wr-Ax*cg*W/L/2;
        IA_f = -L*sin(pitch)/2*IA_gainf + IA_0f;% - KPIf*(1-cos(deltaf)) + casterf*sin(deltaf);
        IA_r = L*sin(pitch)/2*IA_gainr + IA_0r;% - KPIr*(1-cos(deltar)) + casterf*sin(deltar);
        sl = -.2:.01:0;
        for k = 1:length(sl)
            fxf(k) = MF52_Fx_fcn(lCoeff,[-wf -IA_f],sl(k))*sf_x;
            fxr(k) = MF52_Fx_fcn(lCoeff,[-wr -IA_r],sl(k))*sf_x;
        end
        FXF = min(fxf);
        FXR = min(fxr);
        FX = abs(2*FXF+2*FXR);
        AX = FX/W;
        AX_diff = AX-Ax;
    end
    A_X(i) = AX;
    wr_test(i) = wr;
    wf_test(i) = wf;
end
% plot(velocity,wr_test,'r',velocity,wf_test,'b')
% legend('Wr','Wf')
% grid on
% grid minor

velocity_y = lateralg.*9.81.*radii;
velocity_y = sqrt(velocity_y);

r_max = max(radii);
spcount = spcount+1;
shift_points(spcount) = V+1;
top_speed = V;
VMAX = top_speed;
% make the rest of your functions for the GGV diagram
% braking as a function of speed
deccel = polyfit(velocity,A_X,4);
% lateral g's as a function of velocity
lateral = polyfit(velocity_y,lateralg,4);
radii = velocity_y.^2./lateralg/9.81;
% max velocity as a function of instantaneous turn radius
cornering = polyfit(radii,velocity_y,4);

if showPlots == true
figure
t = tiledlayout(2,3);
title(t, "CG Height: " + cg + "m ("+ cg*39.37 +" inches)")
nexttile
range = linspace(4.5,30);
plot(velocity,A_xr,'o',range,polyval(grip,range))
title('Tire limited and Grip limited Acceleration vs Velocity')
grid on
grid minor
hold on
fnplt(accel)
legend('Tire Limited','Grip Limited')
nexttile
plot(velocity,A_X,'o',range,polyval(deccel,range))
title('Braking Acceleration vs Velocity')
grid on
grid minor
nexttile
range = linspace(4.5,velocity_y(end));
plot(velocity_y,lateralg,'o',range,polyval(lateral,range))
title('Lateral Acceleration vs Velocity')
hold on
grid on
grid minor
nexttile
range = linspace(radii(1),radii(end));
plot(radii,velocity_y,'o',range,polyval(cornering,range))
title('Velocity vs Radius')
grid on
grid minor
end

%% Section 7: Load Endurance Track Coordinates
disp('Loading Endurance Track Coordinates')
% [data text] = xlsread('Endurance_Coordinates_1_SI.xlsx','Scaled');
% 
% % the coordinates are now contained within 'data'. This is a 5 column
% % matrix that contains a set of defined 'gates' that the car must mavigate
% % through
% % Column 1: Gate #
% % Column 2: Outside boundary, x coordinate
% % Column 3: Outside boundary, y coordinate
% % Column 4: Inside boundary, x coordinate
% % Column 5: Inside boundary, y coordinate
% 
% % sort the data into "inside" and "outside" cones
% outside = data(:,2:3);
% inside = data(:,4:5);
% t = [1:length(outside)];
% % define the minimum turn radius of the car
% r_min = 4.5;
% r_min = r_min-tw/2;
% pp_out = spline(t,outside');
% pp_in = spline(t,inside');
% 
% for i = 1:1:length(outside)
%     % isolate individual gates
%     gate_in = inside(i,:);
%     gate_out = outside(i,:);
%     % create the line that connects the two cones together
%     x1 = gate_in(1);
%     x2 = gate_out(1);
%     y1 = gate_in(2);
%     y2 = gate_out(2);
%     % polynomial expression for the line:
%     coeff = polyfit([x1, x2], [y1, y2], 1);
%     % adjust the width of the gate for the width of the car:
%     gate_width = sqrt((x2-x1)^2+(y2-y1)^2);
%     path_width = gate_width-tw;
%     x_fs = tw/(2*gate_width);
%     % update the gate boundaries based on said new width
%     x_bound = [min(x1,x2)+x_fs*abs(x2-x1),max(x1,x2)-x_fs*abs(x2-x1)];
%     path_boundaries(i,:) = [coeff x_bound];
% end
%% Section 8: Load Endurance Racing Line
% disp('Loading Endurance Racing Line')
% xx = load('endurance_racing_line.mat');
% xx = xx.endurance_racing_line;
% Section 9: Optimize Endurance Racing Line
% The pre-loaded racing line should work for most applications; however,
% if you have the need to re-evaluate or generate a new optimized racing
% line, simply un-comment the code below:

% 
% disp('Optimizing Endurance Racing Line')
% A = eye(length(xx));
% b = ones(length(xx),1);
% lb = zeros(1,length(xx));
% ub = ones(1,length(xx));
% options = optimoptions('fmincon',...
%     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-12);
% options = optimoptions(options,'MaxIter', 10000, 'MaxFunEvals', 1000000,'ConstraintTolerance',1e-12,'DiffMaxChange',.1);
% 
% x = fmincon(@lap_time,xx,[],[],[],[],lb,ub,@track_curvature,options);
% xx = x;
% x(end+1) = x(1);
% x(end+1) = x(2);

%% Section 10: Generate Final Endurance Trajectory
% x = xx;
% % Plot finished line
% x(end+1) = x(1);
% x(end+1) = x(2);
% for i = 1:1:length(x)
%     % for each gate, find the position defined between the cones
%     coeff = path_boundaries(i,1:2);
%     x2 = max(path_boundaries(i,3:4));
%     x1 = min(path_boundaries(i,3:4));
%     position = x(i);
%     % place the car within via linear interpolation
%     x3 = x1+position*(x2-x1);
%     y3 = polyval(coeff,x3);
%     %plot(x3,y3,'og')
%     % the actual car's trajectory defined in x-y coordinates:
%     path_points(i,:) = [x3 y3];
% end
% 
% x = linspace(1,t(end-1),1000);
% ppv = pchip(t,path_points');
% vehicle_path = ppval(ppv,x);
% vehicle_path_EN = vehicle_path;
% Length = arclength(vehicle_path(1,:),vehicle_path(2,:));


%%%%%%custom track
track = readtable('Track Creation/17_lincoln_endurance_track.xls');

% t = 1:height(track);
% path_points = [track.X, track.Y]'/1000;
% x = linspace(1,t(end-1),1000);
% ppv = pchip(t,path_points);
% vehicle_path = ppval(ppv,x);
% fnplt(ppv)

%% Section 11: Simulate Endurance Lap
%disp('Plotting Vehicle Trajectory')
[laptime time_elapsed velocity acceleration lateral_accel gear_counter path_length weights distance] = lap_information(track, showPlots);

%% Section 12: Load Autocross Track Coordinates
% %disp('Loading Autocross Track Coordinates')
% [data text] = xlsread('Autocross_Coordinates_2_SI.xlsx','Scaled');
% outside = data(:,2:3);
% inside = data(:,4:5);
% t = [1:length(outside)];
% pp_out = spline(t,outside');
% pp_in = spline(t,inside');
% 
% %plot(outside(:,1),outside(:,2),'ok')
% %hold on
% %plot(inside(:,1),inside(:,2),'ok')
% clear path_boundaries
% for i = 1:1:length(outside)
%     gate_in = inside(i,:);
%     gate_out = outside(i,:);
%     %plot([gate_in(1) gate_out(1)],[gate_in(2) gate_out(2)],'-k')
%     x1 = gate_in(1);
%     x2 = gate_out(1);
%     y1 = gate_in(2);
%     y2 = gate_out(2);
%     coeff = polyfit([x1, x2], [y1, y2], 1);
%     gate_width = sqrt((x2-x1)^2+(y2-y1)^2);
%     path_width = gate_width-tw;
%     x_fs = tw/(2*gate_width);
%     x_bound = [min(x1,x2)+x_fs*abs(x2-x1),max(x1,x2)-x_fs*abs(x2-x1)];
%     path_boundaries_ax(i,:) = [coeff x_bound];
%     %text(round(x1),round(y1),num2str(i))
% end


%save('path_boundaries.mat','path_boundaries');
%% Section 13: Load Autocross Racing Line
% %disp('Loading Autocross Racing Line')
% xx = load('autocross_racing_line.mat');
% xx = xx.autocross_racing_line;
% %% Section 14: Optimize Autocross Racing Line
% % Same applies here, optimizing the line is optional but if you want,
% % simply un-comment the lines of code below:
% 
% 
% % disp('Optimizing Racing Line')
% % A = eye(length(xx));
% % b = ones(length(xx),1);
% % lb = zeros(1,length(xx));
% % ub = ones(1,length(xx));
% % options = optimoptions('fmincon',...
% %     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-12);
% % options = optimoptions(options,'MaxIter', 10000, 'MaxFunEvals', 1000000,'ConstraintTolerance',1e-12,'DiffMaxChange',.1);
% % 
% % x = fmincon(@lap_time_sprint,xx,[],[],[],[],lb,ub,@track_curvature_sprint,options);
% % xx_auto = x;
% % % x(end+1) = x(1);
% % % x(end+1) = x(2);
% %% Section 15: Generate Final Autocross Trajectory
% xx_auto = xx;
% x = xx_auto;
% %Plot finished line
% 
% for i = 1:1:length(x)
%     coeff = path_boundaries_ax(i,1:2);
%     x2 = max(path_boundaries_ax(i,3:4));
%     x1 = min(path_boundaries_ax(i,3:4));
%     position = x(i);
%     x3 = x1+position*(x2-x1);
%     y3 = polyval(coeff,x3);
%     %plot(x3,y3,'og')
%     path_points_ax(i,:) = [x3 y3];
% end
% x = linspace(1,t(end),1000);
% ppv = pchip(t,path_points_ax');
% vehicle_path = ppval(ppv,x);
% vehicle_path_AX = vehicle_path;
% Length = arclength(vehicle_path(1,:),vehicle_path(2,:));
% %% Section 16: Simulate Autocross Lap
% %disp('Plotting Vehicle Trajectory')
% [laptime_ax time_elapsed_ax velocity_ax, acceleration_ax lateral_accel_ax gear_counter_ax path_length_ax weights_ax distance_ax] = lap_information_sprint(xx_auto);
%% Section 17: Calculate Dynamic Event Points
%disp('Calculating Points at Competition')
% calculate endurance score
Tmin = 115.249;
Tmax = Tmin*1.45;
Endurance_Score =250*((Tmax/(laptime+13))-1)/(Tmax/Tmin-1)+25;

% % Calculate autocross score
% Tmin = 48.799;
% Tmax = Tmin*1.45;
% Autocross_Score =118.5*((Tmax/(laptime_ax))-1)/(Tmax/Tmin-1)+6.5;

% Skidpad Analysis
% define skidpad turn radius (m)
path_radius = 7.62+tw/2+.1524;
% determine speed possible to take:
speed = polyval(cornering,path_radius);
% calculate skidpad time
skidpad_time = path_radius*2*pi/speed;
% calculate score based on 2019 times
Tmin_skid = 4.865;
Tmax_skid = Tmin_skid*1.45;
Skidpad_Score = 71.5*((Tmax_skid/skidpad_time)^2-1)/((Tmax_skid/Tmin_skid)^2-1) + 3.5;

% Acceleration Analysis
% start at speed 0, gear 1, etc
count = 0;
v = 0;
vel = v;
gears = find((shift_points-vel)>0);
gear = gears(1)-1;
newgear = gear;
time_shifting = 0;
interval = 1;
% accel track is 247 feet, so I am defining 247 segments of 1 foot:
segment = 1:.3048:247*.3048;
t_accel_elapsed = 0;
clear dt_f v_f
% little quickie accel sim:
for i = 1:1:length(segment)
    d = .3048;

    %gear = newgear;
    % find what gear you are in
    gears = find((shift_points-vel)>0);
    newgear = gears(1)-1;
    
    % compare to previous iteration, to detect an upshift
    if newgear > gear
        shifting = 1;
    else
        shifting = 0;
    end

    vmax = VMAX;
    % determine instantaneous acceleration capacity
    AX = fnval(accel,vel);
    dd = d/interval;
    for j = 1:1:interval
        count = count+1;
        vehicle_gear(count) = gear;
        if shifting == 1 & vel < vmax;
            % if you are shifting, then you are not accelerating, but
            % continue to travel forward at constant velocity
            dt_f(count) = dd/vel;
            time_shifting = time_shifting+dt_f(count);
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
            vel = vel;
        elseif vel < vmax
            % if you are not shifting, and top speed has not been achieved
            % then you keep accelerating at maximum capacity possible
            ax_f(count) = AX;
            tt = roots([0.5*9.81*ax_f(count) vel -dd]);
            dt_f(count) = max(tt);
            dv = 9.81*ax_f(count)*dt_f(count);
            dvmax = vmax-vel;
            dv_f(count) = min(dv,dvmax);
            v_f(count) = vel+dv_f(count); 
            vel = v_f(count);
            gears = find((shift_points-vel)>0);
            newgear = gears(1)-1;
            if newgear > gear
                shifting = 1;
            end
        else
            % if you are not shifting but you are at top speed, then just
            % hold top speed
            vel = vmax;
            dt_f(count) = dd/vel;
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
        end
        if time_shifting > shift_time
            shifting = 0;
            time_shifting = 0;
            gear = newgear;
        end
    end
    if shifting == 1
        gear = gear;
    else
        gear = newgear;
    end
    t_accel_elapsed = t_accel_elapsed+dt_f(count);
    t_accel(i) = t_accel_elapsed;
end
accel_time = sum(dt_f(2:end))+.1;
% calculate accel score:
Tmin_accel = 4.109;
Tmax_accel = Tmin_accel*1.5;
Accel_Score = 95.5*((Tmax_accel/accel_time)-1)/((Tmax_accel/Tmin_accel)-1) + 4.5;

% Total_Points = Accel_Score+Skidpad_Score+Autocross_Score+Endurance_Score;

results = [time_elapsed' velocity'];
% xlswrite('logged_data.xlsx',results,'sim_data') 
%% Section 18: Generate Load Cases
%disp('Generating Load Cases')
% find all three worst case acceleration cases:
AX_min = min(acceleration);
AX_max = max(acceleration);
AY_max = max(lateral_accel);
% then find where they took place
VX_min = velocity(find(acceleration == AX_min));
VX_max = velocity(find(acceleration == AX_max));
VY_max = velocity(find(lateral_accel == AY_max));
VY_max = max(VY_max);
frontF = zeros(3,3);
rearF = zeros(3,3);
% then calculate loads based on those speeds and accelerations: 
% see documentation spreadsheet for translation
% frontF(3,:) = [WF/2 + Cl*VX_max^2*CoP/2 - WF*AX_max*cg/L/2 , WF/2 + Cl*VX_min^2*CoP/2 - WF*AX_min*cg/L/2, WF/2 + Cl*VY_max^2*CoP/2 + WF*AY_max*cg/tw/2];
% rearF(3,:) = [WR/2 + Cl*VX_max^2*(1-CoP)/2 + WR*AX_max*cg/L/2 , WR/2 + Cl*VX_min^2*(1-CoP)/2 + WR*AX_min*cg/L/2, WR/2 + Cl*VY_max^2*(1-CoP)/2 + WR*AY_max*cg/tw/2];
% frontF(2,:) = [0 0 (WF/2+WF*AY_max*cg/tw/2)*AY_max];
% rearF(2,:) = [0 0 (WR/2+WR*AY_max*cg/tw/2)*AY_max];
% frontF(1,:) = [0 -(WF/2 -WF*AX_min*cg/L/2)*AX_min 0];
% rearF(1,:) = [W*AX_max/2 -(WR/2 +WR*AX_min*cg/L/2)*AX_min 0];

output = struct('laptime',laptime,'time_elapsed',time_elapsed,'velocity',velocity,'acceleration',acceleration,'lateral_accel' ...
    ,lateral_accel,'gear_counter',gear_counter,'path_length',path_length,'weights',weights,'distance',distance, 'accel_time' ...
    ,accel_time, 'Endurance_Score',Endurance_Score, 'Accel_Score', Accel_Score, 'Skidpad_Score',Skidpad_Score);
% output = [laptime time_elapsed velocity acceleration lateral_accel gear_counter path_length weights distance laptime_ax time_elapsed_ax velocity_ax, acceleration_ax lateral_accel_ax gear_counter_ax path_length_ax weights_ax distance_ax accel_time Endurance_Score Autocross_Score Accel_Score Skidpad_Score];
