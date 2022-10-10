clear all
% clc
% Vechicle Paramaters

%% Section 1: Vehicle Architecture
disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = 51.5; % Front lateral load transfer distribution (%)
W = 650; % vehicle + driver weight (lbs)
WDF = 45; % front weight distribution (%)
cg = 50/12; % center of gravity height (ft)
l = 60.63/12; % wheelbase (ft)
twf = 50.5/12; % front track width (ft)
twr = 48.5/12; % rear track width (ft)

%% Section 2: Input Suspension Kinematics
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
%% Section 3: Input Aero Parameters
disp('Loading Aero Model')

% Cl = linspace(.03,.6,2);
Cl = .0418; %279/418
% Cd = linspace(.01,.03,2);
Cd = .0184; % .024
CoP = 48; % front downforce distribution (%)
%% Run simulation

LapSimOutput = LapSim(LLTD, W, WDF, cg, l, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
    IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
    casterr, KPIr, Cl, Cd, CoP);

distance = LapSimOutput.distance;
velocity = LapSimOutput.velocity;
acceleration = LapSimOutput.acceleration;
lateral_accel = LapSimOutput.lateral_accel;
distance_ax = LapSimOutput.distance_ax;
velocity_ax = LapSimOutput.velocity_ax;
acceleration_ax = LapSimOutput.acceleration_ax;
lateral_accel_ax = LapSimOutput.lateral_accel_ax;
Endurance_time = LapSimOutput.laptime;
Autocross_time = LapSimOutput.laptime_ax;
Accel_time = LapSimOutput.accel_time;
Endurance_score = LapSimOutput.Endurance_Score;
Autocross_score = LapSimOutput.Autocross_Score;
Accel_score = LapSimOutput.Accel_Score;
Skidpad_score = LapSimOutput.Skidpad_Score;

X = [' Endurance time: ',num2str(Endurance_time)];
disp(X)
Y = [' Autocross time: ',num2str(Autocross_time)];
disp(Y)
Z = [' Accel time: ',num2str(Accel_time)];
disp(Z)
XX = [' Endurance score: ',num2str(Endurance_score)];
disp(XX)
YY = [' Autocross score: ',num2str(Autocross_score)];
disp(YY)
ZZ = [' Accel score: ',num2str(Accel_score)];
disp(ZZ)
A = [' Skidpad score: ',num2str(Skidpad_score)];
disp(A)
%% Section 19: Plot Results
% This is just to make some pretty pictures, feel free to comment this out
figure
t = tiledlayout(2, 2);

nexttile
plot(distance,velocity,'k')
title('Endurance Simulation Velocity Trace')
xlabel('Distance Travelled (d) [ft]')
ylabel('Velocity (V) [ft/s]')

nexttile
plot(distance,acceleration,distance,lateral_accel)
title('Endurance Simulation Acceleration Traces')
xlabel('Distance Travelled (d) [ft]')
ylabel('Acceleration [g]')
legend('Longitudinal','Lateral')

nexttile
plot(distance_ax,velocity_ax,'k')
title('Autocross Simulation Velocity Trace')
xlabel('Distance Travelled (d) [ft]')
ylabel('Velocity (V) [ft/s]')

nexttile
plot(distance_ax,acceleration_ax,distance_ax,lateral_accel_ax)
title('Autocross Simulation Acceleration Traces')
xlabel('Distance Travelled (d) [ft]')
ylabel('Acceleration [g]')
legend('Longitudinal','Lateral')
