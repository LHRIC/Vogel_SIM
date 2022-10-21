clear all
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
% clc
% Vechicle Paramaters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
showPlots = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Section 1: Vehicle Architecture
disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = .515; % Front lateral load transfer distribution (%)
W = 650; % vehicle + driver weight (lbs)
WDF = .45; % front weight distribution (%)
cg = 10; % center of gravity height (in)
L = 60.63/12; % wheelbase (ft)
twf = 50.5/12; % front track width (ft)
twr = 48.5/12; % rear track width (ft)

W = W*4.4482216153; % convert to N
cg = cg/39.37; % conver to m
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
Cl = 1.77; %279/418
% Cd = linspace(.01,.03,2);
Cd = .8; % .024
CoP = .48; % front downforce distribution (%% Run simulation

%% Powertrain Parameters

tqMod = 1;

%% Simulate
% 

% 
%     LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
%         IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
%         casterr, KPIr, Cl, Cd, CoP, tqMod, showPlots);
% 
% distance = LapSimOutput.distance;
% velocity = LapSimOutput.velocity;
% acceleration = LapSimOutput.acceleration;
% lateral_accel = LapSimOutput.lateral_accel;
% Endurance_time = LapSimOutput.laptime;
% Accel_time = LapSimOutput.accel_time;
% Endurance_score = LapSimOutput.Endurance_Score;
% Accel_score = LapSimOutput.Accel_Score;
% Skidpad_score = LapSimOutput.Skidpad_Score;
% 
% X = [' Endurance time: ',num2str(Endurance_time)];
% disp(X)
% Z = [' Accel time: ',num2str(Accel_time)];
% disp(Z)
% XX = [' Endurance score: ',num2str(Endurance_score)];
% disp(XX)
% ZZ = [' Accel score: ',num2str(Accel_score)];
% disp(ZZ)
% A = [' Skidpad score: ',num2str(Skidpad_score)];
% disp(A)
Weight = (525:5:725)*4.4482216153;
for runs = 1:length(Weight)
    W = Weight(runs);
    LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod, showPlots);

distance = LapSimOutput.distance;
velocity = LapSimOutput.velocity;
acceleration = LapSimOutput.acceleration;
lateral_accel= LapSimOutput.lateral_accel;
Endurance_time(runs) = LapSimOutput.laptime;
Accel_time(runs) = LapSimOutput.accel_time;
Endurance_score(runs) = LapSimOutput.Endurance_Score;
Accel_score(runs) = LapSimOutput.Accel_Score;
Skidpad_score(runs) = LapSimOutput.Skidpad_Score;

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

filename = append("C:\GrabCode\Vogel_Sim\Output_Files\WSens_", num2str(W),".mat");

% save(filename, 'LapSimOutput')

end

figure
plot(Weight,Endurance_time,'o',Weight,Endurance_time)
title('Endurance time sensitivity to Weight increase')
xlabel('Weight')
ylabel('Endurance Time')
hold off

figure
plot(Weight,Accel_time,'o',Weight,Accel_time)

%% Section 19: Plot Results
% This is just to make some pretty pictures, feel free to comment this out
if showPlots == true
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
end


