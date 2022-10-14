clear all
% clc
% Vechicle Paramaters

%% Section 1: Vehicle Architecture
disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = .515; % Front lateral load transfer distribution (%)
W = 650; % vehicle + driver weight (lbs)
WDF = .45; % front weight distribution (%)
cg = 5; % center of gravity height (in)
L = 60.63/12; % wheelbase (ft)
twf = 50.5/12; % front track width (ft)
twr = 48.5/12; % rear track width (ft)

W = W*4.4482216153; % convert to N
cg = cg/39.37; % convert to m
L = L/3.280839895013123; % convert to m
twf = twf/3.280839895013123; % convert to m
twr = twr/3.280839895013123; % convert to m
%% Section 2: Input Suspension Kinematics
disp('Loading Suspension Kinematics')
% this section is actually optional. So if you set everything to zero, you
% can essentially leave this portion out of the analysis. Useful if you are
% only trying to explore some higher level relationships

% Pitch and roll gradients define how much the car's gonna move around
rg_f = 0; % front roll gradient (rad/g)
rg_r = 0; % rear roll gradient (rad/g)
pg = 0; % pitch gradient (rad/g)
WRF = 31522.830300000003; % front and rear ride rates (N/m)
WRR = 31522.830300000003; 



% then you can select your camber alignment
IA_staticf = 0; % front static camber angle (rad)
IA_staticr = 0; % rear static camber angle (rad)
IA_compensationf = 0; % front camber compensation (%)
IA_compensationr = 0; % rear camber compensation (%)

% lastly you can select your kingpin axis parameters
casterf = 0; % front caster angle (deg)
KPIf = 0; % front kingpin inclination angle (deg)
casterr = 4.1568; % rear caster angle (deg)
KPIr = 0; % rear kingpin inclination angle (deg)
%% Section 3: Input Aero Parameters
disp('Loading Aero Model')

% Cl = linspace(.03,.6,2);
Cl = .0418*47.8802589804; %279/418
% Cd = linspace(.01,.03,2);
Cd = .0184*47.8802589804; % .024
CoP = .48; % front downforce distribution 
%% Run simulation


LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
    IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
    casterr, KPIr, Cl, Cd, CoP);


% 
% CG = linspace(9,13.5,20)/39.37;
% 
% for i = 1:length(CG)
% 
%     cg = CG(i);
% 
% LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
%     IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
%     casterr, KPIr, Cl, Cd, CoP);
% 
% Endurance_time(i) = LapSimOutput.laptime;
% 
% end
% 
% plot(CG*39.37,Endurance_time,'r')
% title('CG sensitivity')
% legend('CG height','Endurance laptime')
% grid on
% grid minor

distance = LapSimOutput.distance;
velocity = LapSimOutput.velocity;
acceleration = LapSimOutput.acceleration;
lateral_accel = LapSimOutput.lateral_accel;
Endurance_time = LapSimOutput.laptime;
Accel_time = LapSimOutput.accel_time;
Endurance_score = LapSimOutput.Endurance_Score;
Accel_score = LapSimOutput.Accel_Score;
Skidpad_score = LapSimOutput.Skidpad_Score;

X = [' Endurance time: ',num2str(Endurance_time)];
disp(X)
Z = [' Accel time: ',num2str(Accel_time)];
disp(Z)
XX = [' Endurance score: ',num2str(Endurance_score)];
disp(XX)
ZZ = [' Accel score: ',num2str(Accel_score)];
disp(ZZ)
A = [' Skidpad score: ',num2str(Skidpad_score)];
disp(A)
%% Section 19: Plot Results
% This is just to make some pretty pictures, feel free to comment this out
figure
t = tiledlayout(1, 2);

nexttile
plot(distance,velocity,'k')
title('Endurance Simulation Velocity Trace')
xlabel('Distance Travelled (d) [m]')
ylabel('Velocity (V) [m/s]')

nexttile
plot(distance,acceleration,distance,lateral_accel)
title('Endurance Simulation Acceleration Traces')
xlabel('Distance Travelled (d) [ft]')
ylabel('Acceleration [g]')
legend('Longitudinal','Lateral')


