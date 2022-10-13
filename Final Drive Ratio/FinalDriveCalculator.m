%% SETUP
close all;
clear all;
clc;

g = 9.8;

%% Car Specs

% Vehicle Specs
    Rt = 0.0825; % wheel radius
    redline = 12000;

% Engine Data Import 
    engine_torque_data = readmatrix('Engine Data');
    engine_gear_ratio = [1/10, 2/10, 3/10, 3.5/10, 3.75/10];

    figure();
        plot(engine_torque_data(:,1), engine_torque_data(:,2))

% effictive mass calculation
 effective_mass = 294.835;

final_drive_ratio = 2;

%% Accel Test 

% Redline Trim
redline_I = find(engine_torque_data(:,1)>redline);
engine_torque_data = engine_torque_data(1:redline_I(1),:);

% conversion factors for data
rpm_mps = (2*pi()*Rt)/60; % rpm to m/s
Nm_mps2 = 1/(Rt*effective_mass); % Nm to m/s2

% converts engine torque data to acceleration(m/s2) / velocity(m/s) 
engine_acc_vel = [engine_torque_data(:,1)*rpm_mps, engine_torque_data(:,2)*Nm_mps2];

figure(); hold on

combined_engine_acc_vel = Torque_Curve_Optimizer(engine_acc_vel,engine_gear_ratio,final_drive_ratio,75);


