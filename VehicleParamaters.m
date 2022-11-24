clear all
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
% clc
% Vechicle Paramaters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
showPlots = true;
singlerunOptions = [1 0 0 0 0 0 0 0];  % (1) = single run (2) = Mass Sweep (3) = Cg height Sweep 
% (4) = Weight Distribution Sweep (with CoP) (5) = Track width sweep (6) = Friction Scaling Sweep
% (7) = Lateral Load Transfer Distribution Sweep % (8) = Final Drive

combinedrunOptions = [0 0]; % (1) = LLTD and Weight distribution (2) = CoP and Cgx

% Paramater Sweep Values
Weight = (525:10:725)*4.4482216153;
Cg = (10:.1:13.5)./39.37;
weightDistribution = (.4:.01/5:.5);
tw = (48:.2:50)./39.37;
longitudinalFriction = (.4:.05:.9);
lateralFriction = (.4:.05:.9);
LLTDistribution = (.30:.05:.80);
centerofPressure = (.4:.1/5:.6);
parforvalue = 0;
FD = (25/11:1/11:46/11);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Section 1: Vehicle Architecture
disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = .5; % Front lateral load transfer distribution (%)
W = 650; % vehicle + driver weight (lbs)
WDF = .45; % front weight distribution (%)
cg = 12.5; % center of gravity height (in)
L = 1.535; % wheelbase (m)
twf = 48; % front track width (in)
twr = 48; % rear track width (in)
FD = 37/11;

W = W*4.4482216153; % convert to N
cg = cg/39.37; % convert to m
twf = twf*.0254; % convert to m
twr = twr*.0254; % convert to m

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

sf_x = .6; % Longitudinal and Lateral Friction Scaling
sf_y = .6;

%% Section 3: Input Aero Parameters
disp('Loading Aero Model')

% Cl = linspace(.03,.6,2);
Cl = 1.77; %279/418
% Cd = linspace(.01,.03,2);
Cd = .8; % .024
CoP = .48; % front downforce distribution (%% Run simulation

%% Powertrain Parameters
tqMod = 1;
FD = FD;

%% Single Paramater Sweeps
if singlerunOptions(1) == 1
 

        LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue,FD);

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
end


if singlerunOptions(2) == 1

parfor runs = 1:length(Weight)
    W = Weight(runs);
    
LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue);

Endurance_time(runs) = LapSimOutput(1)
Accel_time(runs) = LapSimOutput(2)

end

figure
plot(Weight./4.4482216153,Endurance_time,'o',Weight./4.4482216153,Endurance_time)
title('Endurance time sensitivity to Weight increase','FontWeight','bold','FontSize',24)
xlabel('Weight (lbs)','FontSize',18)
ylabel('Endurance Time (s)','FontSize',18)
grid on 
grid minor
hold off

figure
plot(Weight./4.4482216153,Accel_time,'o',Weight./4.4482216153,Accel_time)
title('Accel time sensitivty to Weight increase ','FontWeight','bold','FontSize',24)
xlabel('Weight (lbs)','FontSize',18)
ylabel('Accel Time (s)','FontSize',18)
grid on 
grid minor
end

if singlerunOptions(3) == 1 


parfor runs = 1:length(Cg)
    cg = Cg(runs);

LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue);

Endurance_time(runs) = LapSimOutput(1)
Accel_time(runs) = LapSimOutput(2)

end

figure
plot(Cg.*39.37,Endurance_time,'o',Cg.*39.37,Endurance_time)
title('Endurance time sensitivity to Cg Height','FontWeight','bold','FontSize',24)
xlabel('Cg Height (in)','FontSize',18)
ylabel('Endurance Time (s)','FontSize',18)
grid on 
grid minor
hold off

figure
plot(Cg.*39.37,Accel_time,'o',Cg.*39.37,Accel_time)
title('Accel time sensitivty to Cg Height','FontWeight','bold','FontSize',24)
xlabel('Cg Height (in)','FontSize',18)
ylabel('Accel Time (s)','FontSize',18)
grid on 
grid minor

end

if singlerunOptions(4) == 1 


parfor runs = 1:length(weightDistribution)
    WDF = weightDistribution(runs);
    Cop = weightDistribution(runs);

LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue);

Endurance_time(runs) = LapSimOutput(1)
Accel_time(runs) = LapSimOutput(2)
end

figure
plot(weightDistribution,Endurance_time,'o',weightDistribution,Endurance_time)
title('Endurance time sensitivity to Weight Distribution','FontWeight','bold','FontSize',24)
xlabel('Weight Distribution','FontSize',18)
ylabel('Endurance Time (s)','FontSize',18)
grid on 
grid minor
hold off

figure
plot(weightDistribution,Accel_time,'o',weightDistribution,Accel_time)
title('Accel time sensitivty to Weight Distribution','FontWeight','bold','FontSize',24)
xlabel('Weight Distribution','FontSize',18)
ylabel('Accel Time (s)','FontSize',18)
grid on 
grid minor

end


if singlerunOptions(5) == 1 


parfor runs = 1:length(tw)
    twf = tw(runs);
    twr = tw(runs);

LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue);

Endurance_time(runs) = LapSimOutput(1)
Accel_time(runs) = LapSimOutput(2)

end

figure
plot(tw.*39.37,Endurance_time,'o',tw.*39.37,Endurance_time)
title('Endurance time sensitivity to Track Width','FontWeight','bold','FontSize',24)
xlabel('Track Width (in)','FontSize',18)
ylabel('Endurance Time (s)','FontSize',18)
grid on 
grid minor
hold off

figure
plot(tw.*39.37,Accel_time,'o',tw.*39.37,Accel_time)
title('Accel time sensitivty to Track Width','FontWeight','bold','FontSize',24)
xlabel('Track Width (in)','FontSize',18)
ylabel('Accel Time (s)','FontSize',18)
grid on 
grid minor

end

if singlerunOptions(6) == 1 


parfor runs = 1:length(longitudinalFriction)
    sf_x = longitudinalFriction(runs);

LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue);

Endurance_time(runs) = LapSimOutput(1)
Accel_time(runs) = LapSimOutput(2)


end
save('LongitduinalFriction',"Endurance_time", "Accel_time")
figure
plot(longitudinalFriction,Endurance_time,'o',longitudinalFriction,Endurance_time)
title('Endurance time sensitivity to Longitudinal Friction Scaling','FontWeight','bold','FontSize',24)
xlabel('Longitdinal Friction Scaling','FontSize',18)
ylabel('Endurance Time (s)','FontSize',18)
grid on 
grid minor
hold off

figure
plot(longitudinalFriction,Accel_time,'o',longitudinalFriction,Accel_time)
title('Accel time sensitivty to Longitudinal Friction Scaling','FontWeight','bold','FontSize',24)
xlabel('Longitdinal Friction Scaling','FontSize',18)
ylabel('Accel Time (s)','FontSize',18)
grid on 
grid minor

parfor runs = 1:length(lateralFriction)
    sf_y = lateralFriction(runs);

LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue);

Endurance_time(runs) = LapSimOutput(1)
Accel_time(runs) = LapSimOutput(2)

end

figure
plot(lateralFriction,Endurance_time,'o',lateralFriction,Endurance_time)
title('Endurance time sensitivity to Lateral Friction Scaling','FontWeight','bold','FontSize',24)
xlabel('Longitdinal Friction Scaling','FontSize',18)
ylabel('Endurance Time (s)','FontSize',18)
grid on 
grid minor
hold off

figure
plot(lateralFriction,Accel_time,'o',lateralFriction,Accel_time)
title('Accel time sensitivty to Lateral Friction Scaling','FontWeight','bold','FontSize',24)
xlabel('Longitdinal Friction Scaling','FontSize',18)
ylabel('Accel Time (s)','FontSize',18)
grid on 
grid minor
end

if singlerunOptions(7) == 1 


parfor runs = 1:length(LLTDistribution)
    LLTD = LLTDistribution(runs);

LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue);

Endurance_time(runs) = LapSimOutput(1)
Accel_time(runs) = LapSimOutput(2)
end

figure
plot(LLTDistribution,Endurance_time,'o',LLTDistribution,Endurance_time)
title('Endurance time sensitivity to LLTD','FontWeight','bold','FontSize',24)
xlabel('LLTD','FontSize',18)
ylabel('Endurance Time (s)','FontSize',18)
grid on 
grid minor
hold off

figure
plot(LLTDistribution,Accel_time,'o',LLTDistribution,Accel_time)
title('Accel time sensitivty to LLTD','FontWeight','bold','FontSize',24)
xlabel('LLTD','FontSize',18)
ylabel('Accel Time (s)','FontSize',18)
grid on 
grid minor

end

if singlerunOptions(8) == 1 


parfor runs = 1:length(FD)
    finalDrive = FD(runs)

LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue, finalDrive);

Endurance_time(runs) = LapSimOutput(1)
Accel_time(runs) = LapSimOutput(2)
end

figure
plot(FD,Endurance_time,'o',FD,Endurance_time)
title('Endurance time sensitivity to Final Drive','FontWeight','bold','FontSize',24)
xlabel('Final Drive','FontSize',18)
ylabel('Endurance Time (s)','FontSize',18)
grid on 
grid minor
hold off

figure
plot(FD,Accel_time,'o',FD,Accel_time)
title('Accel time sensitivty to Final Drive','FontWeight','bold','FontSize',24)
xlabel('Final Drive','FontSize',18)
ylabel('Accel Time (s)','FontSize',18)
grid on 
grid minor

end
%% Combination Parameter Sweep 
if combinedrunOptions(1) == 1
parforvalue = 0;
[weightDistribution,LLTDistribution] = meshgrid(weightDistribution,LLTDistribution);

figure(1)
Endurance_time = nan(size(LLTDistribution));
c = mesh(weightDistribution, LLTDistribution, Endurance_time);
colorbar
title('Endurance time sensitivity to LLTD and Weight Distribution','FontWeight','bold','FontSize',24)
xlabel('Weight Distribution','FontSize',18)
ylabel('LLTD','FontSize',18)
zlabel('Endurance Time (s)','FontSize',18)        

D = parallel.pool.DataQueue;
D.afterEach(@(x) updateSurfaceEndurance(c, x));

figure(2)
Accel_Time = nan(size(LLTDistribution));
d = mesh(weightDistribution, LLTDistribution, Accel_Time);
colorbar
title('Accel time sensitivty to LLTD and Weight Distribution','FontWeight','bold','FontSize',24)
xlabel('Weight Distribution','FontSize',18)
ylabel('LLTD','FontSize',18)
zlabel('Accel Time (s)','FontSize',18)        

Q = parallel.pool.DataQueue;
Q.afterEach(@(y) updateSurfaceAccel(d, y));

    parfor ii = 1:numel(weightDistribution)
        LapSimOutput = LapSim(LLTDistribution(ii), W, weightDistribution(ii), cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, CoP, tqMod,showPlots, sf_x,sf_y,parforvalue);

        send(D,[ii LapSimOutput(1)])
        send(Q,[ii LapSimOutput(2)])
    end 
end

if combinedrunOptions(2) == 1
parforvalue = 0;
[weightDistribution,centerofPressure] = meshgrid(weightDistribution,centerofPressure);

figure(1)
Endurance_time = nan(size(centerofPressure));
c = mesh(weightDistribution, centerofPressure, Endurance_time,FaceColor="flat");
colorbar
title('Endurance time sensitivity to COP and Weight Distribution','FontWeight','bold','FontSize',24)
xlabel('Weight Distribution','FontSize',18)
ylabel('COP','FontSize',18)
zlabel('Endurance Time (s)','FontSize',18)        

D = parallel.pool.DataQueue;
D.afterEach(@(x) updateSurfaceEndurance(c, x));

figure(2)
Accel_Time = nan(size(centerofPressure));
d = mesh(weightDistribution, centerofPressure, Accel_Time,FaceColor="flat");
colorbar
title('Accel time sensitivty to COP and Weight Distribution','FontWeight','bold','FontSize',24)
xlabel('Weight Distribution','FontSize',18)
ylabel('COP','FontSize',18)
zlabel('Accel Time (s)','FontSize',18)        

Q = parallel.pool.DataQueue;
Q.afterEach(@(y) updateSurfaceAccel(d, y));

    parfor ii = 1:numel(weightDistribution)
        LapSimOutput = LapSim(LLTD, W, weightDistribution(ii), cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
        IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
        casterr, KPIr, Cl, Cd, centerofPressure(ii), tqMod,showPlots, sf_x,sf_y,parforvalue);

        send(D,[ii LapSimOutput(1)])
        send(Q,[ii LapSimOutput(2)])
    end 
end


function updateSurfaceEndurance(s, d)
s.ZData(d(1)) = d(2);
drawnow('limitrate');
end
function updateSurfaceAccel(s, d)
s.ZData(d(1)) = d(2);
drawnow('limitrate');
end



















