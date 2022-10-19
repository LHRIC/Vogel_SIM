clear all
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
cg = 12.5; % center of gravity height (in)
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

%% MMM Output

vel = 20; % [m/s] velocity to evaluate at (V_long) in velocity frame
steered_angle_max = 10; % [degrees] maximum front steering angle (body frame)
body_angle_max = 10;% [degrees] maximum body angle (velocity frame)
stepsize = 1;

MMMOutput = CalculateMMM(LLTD, W, WDF, cg, L, twf, twr, rg_f, rg_r,pg, WRF, WRR, ...
    IA_staticf, IA_staticr, IA_compensationr, IA_compensationf, casterf, KPIf, ...
    casterr, KPIr, Cl, Cd, CoP, showPlots, vel, steered_angle_max, body_angle_max, stepsize);


%% Section 19: Plot Results
% This is just to make some pretty pictures, feel free to comment this out
disp('Plotting "MMM"')
if showPlots == true
end
% ****** Longitudinal Accel VS C_n Vs Lateral Accel ******
figure
hold
% Plot slip angles
for ii = -steered_angle_max:stepsize:steered_angle_max
    constSlipLine = [];
    for jj = 1:length(MMMOutput(:,1))
        if MMMOutput(jj,2) == ii
            constSlipLine(end+1,:) = MMMOutput(jj,:);
        end
        if ii == 1
            a = constSlipLine;
        end
    end
    plot3(constSlipLine(:,3),constSlipLine(:,7),constSlipLine(:,9),'b')
end

%Plot body slip angles
for ii = -body_angle_max:stepsize:body_angle_max
    constSlipLine = [];
    for jj = 1:length(MMMOutput(:,1))
        if MMMOutput(jj,1) == ii
            constSlipLine(end+1,:) = MMMOutput(jj,:); 
        end
    end
    plot3(constSlipLine(:,3),constSlipLine(:,7),constSlipLine(:,9),'r')
end
title("Yaw Moment Diagram at " + vel + " m/s")
xlabel('Lateral Accel A_y')
ylabel('C_n (M_z/W/L)') % M_z/W/L
zlabel('Longitudinal Accel A_x')
view(3)


% ****** Longitudinal Accel VS Lateral Accel ******
figure
hold
% Plot slip angles
for ii = -steered_angle_max:stepsize:steered_angle_max
    constSlipLine = [];
    for jj = 1:length(MMMOutput(:,1))
        if MMMOutput(jj,2) == ii
            constSlipLine(end+1,:) = MMMOutput(jj,:);
        end
        if ii == 1
            a = constSlipLine;
        end
    end
    plot(constSlipLine(:,9),constSlipLine(:,3),'b')
end

%Plot body slip angles
for ii = -body_angle_max:stepsize:body_angle_max
    constSlipLine = [];
    for jj = 1:length(MMMOutput(:,1))
        if MMMOutput(jj,1) == ii
            constSlipLine(end+1,:) = MMMOutput(jj,:); 
        end
    end
    plot(constSlipLine(:,9),constSlipLine(:,3),'r')
end
title("Longitudinal Accel Vs Lateral Accel at " + vel + " m/s")
xlabel('Longitudinal Accel A_x')
ylabel('Lateral Accel A_y')


% ****** Lateral Accel VS C_n ******
figure
hold
% Plot slip angles
for ii = -steered_angle_max:stepsize:steered_angle_max
    constSlipLine = [];
    for jj = 1:length(MMMOutput(:,1))
        if MMMOutput(jj,2) == ii
            constSlipLine(end+1,:) = MMMOutput(jj,:);
        end
        if ii == 1
            a = constSlipLine;
        end
    end
    plot(constSlipLine(:,3),constSlipLine(:,7),'b')
end

%Plot body slip angles
for ii = -body_angle_max:stepsize:body_angle_max
    constSlipLine = [];
    for jj = 1:length(MMMOutput(:,1))
        if MMMOutput(jj,1) == ii
            constSlipLine(end+1,:) = MMMOutput(jj,:); 
        end
    end
    plot(constSlipLine(:,3),constSlipLine(:,7),'r')
end
title("Yaw Moment Diagram at " + vel + " m/s")
xlabel('Lateral Accel A_y')
ylabel('C_n (M_z/W/L)') % M_z/W/L
% Find Control and Stability
