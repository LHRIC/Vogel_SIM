function [output]=CalculateMMM(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, casterr, KPIr, Cl, Cd, CoP, showPlots, vel, steered_angle_max, body_angle_max, stepsize)
%% Section 0: Name all symbolic variables
% Don't touch this. This is just naming a bunch of variables and making
% them global so that all the other functions can access them
global r_max accel grip deccel lateral cornering gear shift_points...
    top_speed r_min path_boundaries tire_radius shift_time...
    powertrainpackage track_width path_boundaries_ax OptimParameterSet input
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
% *** UNUSED *** I don't yet know of a case with a powertrain model
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
%% Section 6: Generate MMM Diagram
% this is where the m e a t of the lap sim takes place. The MMM diagram is
% built by finding the lateral acceleration and yaw moment at a specified
% velocity
disp('Calculating MMM')
% Legacy from vogel to make work
deltar = 0;
deltaf = 0;
R = 0; % Unused
IA = 0; % Set inclination angle to 0
LF = Cl*vel^2;

%Initialize Data Storage
MMMpoints = [];

for body_angle = -body_angle_max:stepsize:body_angle_max
    for steered_angle = -steered_angle_max:stepsize:steered_angle_max
        % from downforce, update suspension travel (m):
        dxf = LF*CoP/2/WRF; 
        dxr = LF*(1-CoP)/2/WRR; 
        % from suspension heave, update static camber (rad):
        IA_0f = IA_staticf - dxf*IA_gainf; 
        IA_0r = IA_staticr - dxr*IA_gainr; 
        % update load on each wheel (N)
        wf = (WF+LF*CoP)/2;
        wr = (WR+LF*(1-CoP))/2;
        % ADD ACkermann Steering hereish, for now use parallel steer
        % Minimizing the objective function of M_z a_f-12degrees and a_y-AY  
        input = 1;
        AYP = 0; % Lateral acceleration guess
        yaw_rate = 0; % Yaw rate guess
        x0 = [AYP yaw_rate];  % initial values
        fun = @(x)vogelMMM(x,a,b,Cd,IA_gainf,IA_gainr,twf,KPIf,cg,W,twr,LLTD,rg_r, ...
            rg_f,casterf,KPIr,deltar,sf_y,T_lock,R,wf,wr,IA_0f,IA_0r,vel,steered_angle,body_angle);
        fun(x0);
        % minimizing function
        lb = [];
        ub = [];
        opts = optimoptions("lsqnonlin",MaxFunctionEvaluations=10000,MaxIterations=10000,FunctionTolerance=1e-16,Display="off");
        x = lsqnonlin(fun, x0,lb,ub,opts);
        % output from minimizing
        AYP = x(1);
        yaw_rate = x(2);
        input = 0;
        evalVogel = fun(x);
        A_y = evalVogel(1);
        M_z = evalVogel(2);
        a_f = rad2deg(evalVogel(3));
        a_r = rad2deg(evalVogel(4));
        yaw_rate = rad2deg(evalVogel(5));
        yaw_rate = evalVogel(6)
        % Create Data Structure
        MMMpoints(end+1,:) = [body_angle steered_angle A_y M_z a_r a_f M_z/W/L yaw_rate];
    end
end

%% Output
output = MMMpoints;
