%%% LapSim Aero Sweep

clear all;
clc

% Vechicle Paramaters

%% Section 1: Vehicle Architecture
%disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = 0.515; % Front lateral load transfer distribution (%)
WBase = 650; % vehicle + driver weight (lbs)
WDF = .45; % front weight distribution (%)
cg = 12.5; % center of gravity height (ft)
L = 60.63/12; % wheelbase (ft)
twf = 50.5/12; % front track width (ft)
twr = 48.5/12; % rear track width (ft)

WBase = WBase*4.4482216153; % convert to N
cg = cg/39.37; % conver to m
L = L/3.280839895013123; % convert to m
twf = twf/3.280839895013123; % convert to m
twr = twr/3.280839895013123; % convert to m

%% Section 2: Input Suspension Kinematics
%disp('Loading Suspension Kinematics')
% this section is actually optional. So if you set everything to zero, you
% can essentially leave this portion out of the analysis. Useful if you are
% only trying to explore some higher level relationships

% Pitch and roll gradients define how much the car's gonna move around
rg_f = 0; % front roll gradient (deg/g)
rg_r = 0; % rear roll gradient (deg/g)
pg = 0; % pitch gradient (deg/g)
WRF = 31522.830300000003; % front and rear ride rates (N/m)
WRR = 31522.830300000003; 

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
%disp('Loading Aero Model')

CoP = 0.48; % front downforce distribution (%)

CLA = [-2.9:0.1:0.1] * -0.6125; % Lift equation without Velocity component

cdOffsetM = [0:0.05:0.4] * 0.6125;

%% Section 4: Sim Loop

k = 1;

IterTotal = length(CLA) * length(cdOffsetM);

fprintf('Running %.0f iterations.\n', IterTotal);

for j = 1:length(cdOffsetM)
    
    cdOffset = cdOffsetM(j);
    
for i = 1:length(CLA)

    Cl = CLA(i);
    Cd = 0.24 * Cl + (0.3 * 0.6125) + cdOffset;

    W = WBase + (5.38 + 24.7*Cl + 23.7*Cl^2);

LapSimOutput = LapSim(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
    IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
    casterr, KPIr, Cl, Cd, CoP);

T_axismax = max(LapSimOutput.time_elapsed);

ClaM(k) = Cl;
CdaM(k) = Cd;
TimeM(k) = T_axismax;
WeightM(k) = W;

clear LapSimOutput

fprintf('Completed iteration %.0f of %.0f.\n', k, IterTotal);
fprintf('Config with CLA of %.2f, CDA of %.2f, and weight of %.2f resulted with a time of %.2f seconds.\n', (Cl / (-0.6125)), (Cd / (0.6125)), W, T_axismax);

k = k + 1;

end

end

%% Section 5: CoP Sweep

[minTime, BestIndex] = min(TimeM);

ClaBest = ClaM(BestIndex);
CdaBest = ClaM(BestIndex);
WeightBest = WeightM(BestIndex);

CoPRange = [0.25:0.05:0.75];

IterTotalCoP = length(CoPRange);

fprintf('Running %.0f iterations.\n', IterTotalCoP);

for kCoP = 1:length(CoPRange)
    
    CoPC = CoPRange(kCoP);

    LapSimOutput = LapSim(LLTD, WeightBest, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, casterr, KPIr, ClaBest, CdaBest, CoPC);

    T_axismax = max(LapSimOutput.time_elapsed);

    TimeMCoP(kCoP) = T_axismax;

    clear LapSimOutput

    fprintf('Completed iteration %.0f of %.0f.\n', kCoP, IterTotalCoP);
    fprintf('Config with CLA of %.2f, CDA of %.2f, and weight of %.2f resulted with a time of %.2f seconds.\n', (ClaBest / (-0.6125)), (CdaBest / (0.6125)), W, T_axismax);

end

%% Section 6: CoP + CG Sweep

CGRange = [0.50:0.02:0.60];

IterTotalCoPCG = length(CoPRange) * length(CGRange);

kCoPCG = 1;

for iCG = 1:length(CGRange)

    CG = CGRange(iCG);

for iCoPG = 1:length(CoPRange)
    
    CoPCG = CoPRange(iCoPG);

    LapSimOutput = LapSim(LLTD, WeightBest, WDF, CG, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, casterr, KPIr, ClaBest, CdaBest, CoPCG);

    T_axismax = max(LapSimOutput.time_elapsed);

    TimeMCoPG(kCoPCG) = T_axismax;

    CoPM(kCoPCG) = CoPCG;
    CGM(kCoPCG) = CG;

    clear LapSimOutput

    fprintf('Completed iteration %.0f of %.0f.\n', kCoPCG, IterTotalCoPCG);
    fprintf('Config with CLA of %.2f, CDA of %.2f, and weight of %.2f resulted with a time of %.2f seconds.\n', (ClaBest / (-0.6125)), (CdaBest / (0.6125)), W, T_axismax);

    kCoPCG = kCoPCG + 1;

end

end
%% Section 7: Balance Sweep
% set lower Cl (like -2.5) at the best CoP and CG combo. Then gradually
% increase Cl and move CoP forward to see how times change (representing
% increasing fwng performance)
[minTimeCoP, BestIndexCoP] = min(TimeMCoP);

CoPBest = CoPRange(BestIndexCo);

%% Section 000: Data Visualization

figure(1)
xlin = linspace(min(ClaM), max(ClaM), 100);
ylin = linspace(min(CdaM), max(CdaM), 100);
[X,Y] = meshgrid(xlin, ylin);
Z = griddata(ClaM,CdaM,TimeM,X,Y,'v4');
[dfdx, dfdy] = gradient(Z);
s = mesh(X/(-0.6125),Y/(0.6125),Z, sqrt(dfdx.^2 + dfdy.^2));
colormap(turbo);
axis tight; hold on
plot3(ClaM/(-0.6125),CdaM/(0.6125),TimeM,'m.','MarkerSize',5)
s.FaceColor = 'interp';
s.EdgeColor = 'none';
xlabel('Cla')
ylabel('Cda')
zlabel('Time(s)')
g = colorbar;
title(g, 'gradient')
clim([0 0.1])
hold off

figure(2)
plot(CoPRange, TimeMCoP(:, 1), 'b-')
hold on
plot(CoPRange, TimeMCoPAX(:, 1), 'b--')
plot(CoPRange, TimeMCoP(:, 2), 'g-')
plot(CoPRange, TimeMCoPAX(:, 2), 'g--')
plot(CoPRange, TimeMCoP(:, 3), 'm-')
plot(CoPRange, TimeMCoPAX(:, 3), 'm--')
title('Time vs. Center of Pressure');
subtitle('at various CL-CD combinations');
xlabel('CoP Location (% front distribution)')
ylabel('Time')
legend('Best CL-CD Endurance', 'Best CL-CD AutoX','CL=-2, CD=1 Endurance','CL=-2, CD=1 AutoX', 'CL=-1.5, CD=0.85 Endurance','CL=-1.5, CD=0.85 AutoX');

figure(3)
CoPlin = linspace(min(CoPRange), max(CoPRange), 100);
CGlin = linspace(min(CGRange), max(CGRange), 100);
[XCoP,YCG] = meshgrid(CoPlin, CGlin);
ZCoPCG = griddata(CoPRange,CGRange,TimeMCoPCG,XCoP,YCG,'v4');
[dfdxCoPCG, dfdyCoPCG] = gradient(ZCoPCG);
s = mesh(XCoP,YCG,ZCoPCG, sqrt(dfdxCoPCG.^2 + dfdyCoPCG.^2));
colormap(turbo);
axis tight; hold on
plot3(CoPRange,CGRange,TimeMCoPCG,'m.','MarkerSize',5)
s.FaceColor = 'interp';
s.EdgeColor = 'none';
title('CoP-CG-Laptime');
subtitle('Endurance');
xlabel('CoP')
ylabel('CG')
zlabel('Time(s)')
g = colorbar;
title(g, 'gradient')
clim([0 0.1])
hold off

figure(4)
CoPlinAX = linspace(min(CoPRange), max(CoPRange), 100);
CGlinAX = linspace(min(CGRange), max(CGRange), 100);
[XCoPAX,YCGAX] = meshgrid(CoPlinAX, CGlinAX);
ZCoPCGAX = griddata(CoPRange,CGRange,TimeMCoPCGAX,XCoPAX,YCGAX,'v4');
[dfdxCoPCGAX, dfdyCoPCGAX] = gradient(ZCoPCGAX);
s = mesh(XCoPAX,YCGAX,ZCoPCGAX, sqrt(dfdxCoPCGAX.^2 + dfdyCoPCGAX.^2));
colormap(turbo);
axis tight; hold on
plot3(CoPRange,CGRange,TimeMCoPCGAX,'m.','MarkerSize',5)
s.FaceColor = 'interp';
s.EdgeColor = 'none';
title('CoP-CG-Laptime');
subtitle('Autocross');
xlabel('CoP')
ylabel('CG')
zlabel('Time(s)')
g = colorbar;
title(g, 'gradient')
clim([0 0.1])
hold off