%% Load the data
% First load the data into the workspace For this example, the data is stored 
% as a Table and has been already filtered, cropped and pre-processed. The data 
% in this example is already in ISO-W and all channels in SI units (N, Nm, m, 
% s, rad, Pa)

clear all
load('B2356run4.mat') %import TTC Data
scatter(1:length(FZ),FZ,'o') %Create scatter plot for visulization
% 
A = [ET V*.278 N*.10472 SA*pi/180 IA*pi/180 RL RL P*1000 FX FY FZ*-1 MX MZ SR*0 SL]; %Turn data into array
TyreData = array2table(A, 'VariableNames', {'Time' 'Vx' 'W' 'SA' 'IA' 'RI' 'Re' 'P' 'Fx' 'Fy' 'Fz' 'Mx' 'Mz' 'SR' 'Phit'});

%TyreData([1:1246,19984:21227,39963:41212],:)=[]; %delete uneccesary data points

plot(TyreData.SA*180/pi, TyreData.Fy,'o')
grid 
%% Fitting process

% Load a TIR file or an intermediate iteration as a starting point for the fitting
InitalParameterSet = mfeval.readTIR('FSAE_Defaults.tir');
% load('Optimum_Paramater_Set_Iteration_1.mat');
% InitalParameterSet = OptimParameterSet;

% Set nominal parameters of the model (DO NOT CHANGE AFTER)
InitalParameterSet.UNLOADED_RADIUS = 0.20574; % Unloaded tire radius
InitalParameterSet.FNOMIN = 800; % Nominal load
InitalParameterSet.LONGVL = 11.1; % Nominal reference speed
InitalParameterSet.NOMPRES = 82737.1; % Nominal inflation pressure

% Create the initial parameters for the fitting (seeds)
x0(1)  = InitalParameterSet.PCY1;   %Shape factor Cfy for lateral forces
x0(2)  = InitalParameterSet.PDY1;   %Lateral friction Muy
x0(3)  = InitalParameterSet.PDY2;   %Variation of friction Muy with load
x0(4)  = InitalParameterSet.PDY3;   %Variation of friction Muy with squared camber
x0(5)  = InitalParameterSet.PEY1;   %Lateral curvature Efy at Fznom
x0(6)  = InitalParameterSet.PEY2;   %Variation of curvature Efy with load
x0(7)  = InitalParameterSet.PEY3;   %Zero order camber dependency of curvature Efy
x0(8)  = InitalParameterSet.PEY4;   %Variation of curvature Efy with camber
x0(9)  = InitalParameterSet.PEY5;   %Variation of curvature Efy with camber squared
x0(10) = InitalParameterSet.PKY1;   %Maximum value of stiffness Kfy/Fznom
x0(11) = InitalParameterSet.PKY2;   %Load at which Kfy reaches maximum value
x0(12) = InitalParameterSet.PKY3;   %Variation of Kfy/Fznom with camber
x0(13) = InitalParameterSet.PKY4;   %Curvature of stiffness Kfy
x0(14) = InitalParameterSet.PKY5;   %Peak stiffness variation with camber squared
x0(15) = InitalParameterSet.PKY6;   %Fy camber stiffness factor
x0(16) = InitalParameterSet.PKY7;   %Vertical load dependency of camber stiffness
x0(17) = InitalParameterSet.PHY1;   %Horizontal shift Shy at Fznom
x0(18) = InitalParameterSet.PHY2;   %Variation of shift Shy with load
x0(19) = InitalParameterSet.PVY1;   %Vertical shift in Svy/Fz at Fznom
x0(20) = InitalParameterSet.PVY2;   %Variation of shift Svy/Fz with load
x0(21) = InitalParameterSet.PVY3;   %Variation of shift Svy/Fz with camber
x0(22) = InitalParameterSet.PVY4;   %Variation of shift Svy/Fz with camber and load
x0(23) = InitalParameterSet.PPY1;   %influence of inflation pressure on cornering stiffness
x0(24) = InitalParameterSet.PPY2;   %influence of inflation pressure on dependency of nominal tyre load on cornering stiffness
x0(25) = InitalParameterSet.PPY3;   %linear influence of inflation pressure on lateral peak friction
x0(26) = InitalParameterSet.PPY4;   %quadratic influence of inflation pressure on lateral peak friction
x0(27) = InitalParameterSet.PPY5;   %Influence of inflation pressure on camber stiffness

% Declare the anonymous function (Cost function) for the fitting
% The @ operator creates the handle, and the parentheses () immediately
% after the @ operator include the function input arguments
fun = @(X) costFyPure(X, TyreData, InitalParameterSet);

% Options for the fitting function lsqnonlin
options.TolFun = 1e-4; % Low tolerance to ensure good fitting
options.MaxFunEvals = 99999; % Very high to avoid this stop criteria

% Non-linear least squares fitting formula
% lsqnonlin will try to minimize the output of the cost function (error).
% Go to the cost function "costFyPure" to check how this is performed
X_OPTIM = lsqnonlin(fun,x0,[],[],options);

% Create a copy of the initial parameters and replace the fitted parameters
OptimParameterSet = InitalParameterSet;

OptimParameterSet.PCY1 = X_OPTIM(1);
OptimParameterSet.PDY1 = X_OPTIM(2);
OptimParameterSet.PDY2 = X_OPTIM(3);
OptimParameterSet.PDY3 = X_OPTIM(4);
OptimParameterSet.PEY1 = X_OPTIM(5);
OptimParameterSet.PEY2 = X_OPTIM(6);
OptimParameterSet.PEY3 = X_OPTIM(7);
OptimParameterSet.PEY4 = X_OPTIM(8);
OptimParameterSet.PEY5 = X_OPTIM(9);
OptimParameterSet.PKY1 = X_OPTIM(10);
OptimParameterSet.PKY2 = X_OPTIM(11);
OptimParameterSet.PKY3 = X_OPTIM(12);
OptimParameterSet.PKY4 = X_OPTIM(13);
OptimParameterSet.PKY5 = X_OPTIM(14);
OptimParameterSet.PKY6 = X_OPTIM(15);
OptimParameterSet.PKY7 = X_OPTIM(16);
OptimParameterSet.PHY1 = X_OPTIM(17);
OptimParameterSet.PHY2 = X_OPTIM(18);
OptimParameterSet.PVY1 = X_OPTIM(19);
OptimParameterSet.PVY2 = X_OPTIM(20);
OptimParameterSet.PVY3 = X_OPTIM(21);
OptimParameterSet.PVY4 = X_OPTIM(22);
OptimParameterSet.PPY1 = X_OPTIM(23);
OptimParameterSet.PPY2 = X_OPTIM(24);
OptimParameterSet.PPY3 = X_OPTIM(25);
OptimParameterSet.PPY4 = X_OPTIM(26);
OptimParameterSet.PPY5 = X_OPTIM(27);
%% Plot results

% Filter data to plot specific conditions:
indFz1 = TyreData.Fz > 200 & TyreData.Fz < 900;   % 500 N
indFz2 = TyreData.Fz > 400 & TyreData.Fz < 1300;   % 800 N
indFz3 = TyreData.Fz > 100 & TyreData.Fz < 700;    % 300 N
indIA = TyreData.IA > -0.01 & TyreData.IA < 0.01;   % 0 rad
indP = TyreData.P > 8e4 & TyreData.P < 9e4;         % 83160 Pa
indFz = indFz1 | indFz2 | indFz3;
filt = indFz & indIA & indP;

% Create data inputs to do a data replay with MFeval and check the fitting
% quality

% choose parameter to vary, choose values
evalparam = 'Fz [N]';
%values = [300 254 1012];
values = [213 658 882];
% generate matrix for plot
eval = ones(100, length(values));
for j = 1:width(eval)
    eval(:,j) = eval(:,j)*values(j);
end

evalFzlinspace = linspace(300,800)';
evalGamma = linspace(-.05,.05)';
evalNull = zeros(100, 1);
evalSA = linspace(-0.23,0.23)';
evalVx = ones(100, 1)*16;
evalP = ones(100,1)*82737.1;

% inputsMF = [Fz kappa alpha gamma phit Vx P* omega*], where
% Fz     = normal load on the tyre  [N]
% kappa  = longitudinal slip        [dimensionless, -1: locked wheel]
% alpha  = side slip angle          [rad]
% gamma  = inclination angle        [rad]
% phit   = turn slip                [1/m]
% Vx     = forward velocity         [m/s]
% P*     = pressure                 [Pa]
% omega* = rotational speed         [rad/s]
 

figure
plot(TyreData.SA, TyreData.Fy,'o')
for m = 1: length(values)
    
    MFinput = [eval(:,m) evalNull evalSA evalNull evalNull evalVx evalP];   %build MFinput
    MFout = mfeval(OptimParameterSet,MFinput,121);                          %call mfeval
    
    hold on
    
   
    % choose outputs to plot (see below)
    xparam = 8;
    yparam = 2;
    plot(MFout(:,xparam), MFout(:,yparam),'-', 'linewidth', 2)
end
title('PureFy fitting2')

%generate string for legend
leg = [""];
for str = 1:length(values)
    if str>1
        leg = [leg; ""];
    end
    leg(str) = append(evalparam, '=', num2str(values(str)));
end
legend(leg(1), leg(2), leg(3), leg(4)) %have to change this to match number of curves
%plot(TyreData.SA*180/pi, TyreData.Fy,'o')
%MFout (:,1) = Fx - Longitudinal Force
%MFout (:,2) = Fy - Lateral Force
%MFout (:,3) = Fz - Normal Force
%MFout (:,4) = Mx - Overturning Moment
%MFout (:,5) = My - Rolling resitance Moment
%MFout (:,6) = Mz - Self aligning moment
%MFout (:,7) = Kappa - Longituinal slip
%MFout (:,8) = Alpha - Side slip angle
%MFout (:,9) = Gamma - Inclination angle
%MFout (:,10) = Phit - Turn slip
%MFout (:,11) = Vcx - Longitudinal Velocity
%MFout (:,12) = P - Pressure
%MFout (:,13) = Re - effective rilling radius
%MFout (:,14) = Rho - Tyre Deflection 
%MFout (:,15) = 2*a - Contact path length 
%MFout (:,16) = t - Pneumatic Trail 
%MFout (:,17) = mux - Longitudinal Friction Coefficient
%MFout (:,18) = muy - Lateral Friction Coefficient
%MFout (:,19) = Omega - Rotational speed
%MFout (:,20) = R1 - Loaded Radius
%MFout (:,21) = 2*b - Contact Partch Width
%MFout (:,22) = Mzr - Residual Torwuq
%MFout (:,23) = Cx - Longitudinal Stiffness
%MFout (:,24) = Cy - Laterial Stiffness
%MFout (:,25) = Cz - Vertical Stiffness
%MFout (:,26) = Kya - Cornering Stiffness 
%MFout (:,27) = Sigmax - Longitidnal Relaxation length
%MFout (:,28) = Sigmay - Lateral Relaxation Length
%MFout (:,29) = InstKya - Instantaneous Cornering Stiffness
%MFout (:,30) = Kxk - Slip Stiffness


MFinput4linspace = [evalFzlinspace evalNull evalSA evalGamma evalNull evalVx evalP];

MFout4linspace = mfeval(OptimParameterSet,MFinput4linspace,121);
MFout5linspace = mfeval(OptimParameterSet,MFinput4linspace,121);

MFoutFyk_FZ_SA = zeros(100,100);
for n=1:100
    for k=1:100
        MFinput4linspace = [evalFzlinspace(k,:) 0 evalSA(n,:) 0 0 16 82737.1];
        MFoutFysingle1 = mfeval(OptimParameterSet,MFinput4linspace,121);
        MFoutFyk_FZ_SA(n,k) = MFoutFysingle1(:,2);
    end
end

MFoutFyk_FZ_Gamma = zeros(100, 100);
for n=1:100
    for k=1:100
        MFinput5linspace = [800 0 evalSA(n,:) evalGamma(k,:) 0 16 82737.1];
        MFoutFysingle2 = mfeval(OptimParameterSet,MFinput5linspace,121);
        MFoutFyk_FZ_Gamma(n,k) = MFoutFysingle2(:,2);
    end
end

[MFout4Fz, MFout4Alpha] = meshgrid(MFout4linspace(:,3),MFout4linspace(:,8));

figure
mesh(MFout4Alpha*180/pi,MFout4Fz,MFoutFyk_FZ_SA)
xlabel('Slip Angle [deg]')
ylabel('Vertical Force [N]')
zlabel('Lateral Force [N]')
colorbar

% figure
% [MFout5Gamma, MFout5Alpha] = meshgrid(MFout5linspace(:,9),MFout5linspace(:,8));
% 
% figure
% mesh(MFout5Alpha*180/pi,MFout5Gamma*180/pi,MFoutFyk_FZ_Gamma)
% xlabel('Slip Angle [deg]')
% ylabel('Inclination Angle [deg]')
% zlabel('Lateral Force [N]')
% colorbar
%% Nested functions

function [ error ] = costFyPure(X, Data, ParameterSet)
%COSTFYPURE calls MFeval and calculates the error between the model and the
%input data.
%
% error = costFyPure(X, Data, ParameterSet)
%
% X: Is a structure that contains the FyPure parameters that are being
%       fitted. X is changing all the time when lsnonlin is calling this
%       function.
% Data: Is a Table that contains the Data d to measure the error
%       of the model that is being fitted.
% ParameterSet: Is a structure of MF6.1 parameters. The parameters are used
%       only to call MFeval without errors.
%
% Example:
% error = costFyPure(Xstructure, TableData, ParameterSet)

% Create the Inputs for MFeval
INPUTS = [Data.Fz Data.SR Data.SA Data.IA Data.Phit Data.Vx Data.P Data.W];

% Select use mode 111. For more info go to the documentation of MFeval
useMode = 111;

% Unpack the parameters that are being fitted and replace them into the
% ParameterSet.
ParameterSet.PCY1	=  X(1)     ;%Shape factor Cfy for lateral forces
ParameterSet.PDY1	=  X(2)     ;%Lateral friction Muy
ParameterSet.PDY2	=  X(3)     ;%Variation of friction Muy with load
ParameterSet.PDY3	=  X(4)  	;%Variation of friction Muy with squared camber
ParameterSet.PEY1	=  X(5)  	;%Lateral curvature Efy at Fznom
ParameterSet.PEY2	=  X(6)   	;%Variation of curvature Efy with load
ParameterSet.PEY3	=  X(7)   	;%Zero order camber dependency of curvature Efy
ParameterSet.PEY4	=  X(8)  	;%Variation of curvature Efy with camber
ParameterSet.PEY5	=  X(9)   	;%Variation of curvature Efy with camber squared
ParameterSet.PKY1	=  X(10)	;%Maximum value of stiffness Kfy/Fznom
ParameterSet.PKY2	=  X(11) 	;%Load at which Kfy reaches maximum value
ParameterSet.PKY3	=  X(12)   	;%Variation of Kfy/Fznom with camber
ParameterSet.PKY4	=  X(13)   	;%Curvature of stiffness Kfy
ParameterSet.PKY5	=  X(14)   	;%Peak stiffness variation with camber squared
ParameterSet.PKY6	=  X(15)   	;%Fy camber stiffness factor
ParameterSet.PKY7	=  X(16)   	;%Vertical load dependency of camber stiffness
ParameterSet.PHY1	=  X(17)  	;%Horizontal shift Shy at Fznom
ParameterSet.PHY2	=  X(18)   	;%Variation of shift Shy with load
ParameterSet.PVY1	=  X(19)  	;%Vertical shift in Svy/Fz at Fznom
ParameterSet.PVY2	=  X(20)   	;%Variation of shift Svy/Fz with load
ParameterSet.PVY3	=  X(21)   	;%Variation of shift Svy/Fz with camber
ParameterSet.PVY4	=  X(22)  	;%Variation of shift Svy/Fz with camber and load
ParameterSet.PPY1	=  X(23)   	;%influence of inflation pressure on cornering stiffness
ParameterSet.PPY2	=  X(24)  	;%influence of inflation pressure on dependency of nominal tyre load on cornering stiffness
ParameterSet.PPY3	=  X(25)   	;%linear influence of inflation pressure on lateral peak friction
ParameterSet.PPY4	=  X(26)   	;%quadratic influence of inflation pressure on lateral peak friction
ParameterSet.PPY5	=  X(27)   	;%Influence of inflation pressure on camber stiffness

% Call MFeval
OUTPUT = mfeval(ParameterSet,INPUTS,useMode);

% Get the Fy from the MF6.1 model
Fy_MFeval = OUTPUT(:,2);

% Calculate error against the data
error = (Data.Fy - Fy_MFeval);
end