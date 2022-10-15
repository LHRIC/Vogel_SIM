%% SETUP
close all;
clear all;
clc;

g = 9.81;

%% Car Specs

% Vehicle Parameters
Rt = 0.2032; % wheel radius
effective_mass = 294.835;
redline = 12000; % desired maximum rpm
f_max = 10000;

Cr0 = 0.0125; % Coefficient of rolling resistance for tire (velocity independent)
Crp = 2.5*(10.^-4);  % Velocity dependent Coeff of rolling resistance modifier
C_drag = 1.1; % Coefficient of drag 
C_down = 2.9; % Coefficient of lift (downwards)

% Engine Data 
engine_torque_data = readmatrix('Inline_Torque.csv');
engine_primary_reduction = 2.11;
engine_gear_ratio = [33/12, 32/16, 30/18, 26/18, 30/23, 29/24];

% Plots Engine Data
figure();
    plot(engine_torque_data(:,1), engine_torque_data(:,2))
    title("Engine Torque Curve")
    xlabel("RPM")
    ylabel("Torque (Nm)")

% Final Drive Ratio
final_drive_ratio = 37/11;

% Variable Packaging
vehicle_parameters = [f_max, effective_mass, final_drive_ratio]; 
coeff_inputs = [C_down; C_drag; Cr0; Crp]; 

%% Accel Test 
engine_gear_ratio = engine_gear_ratio .* engine_primary_reduction;

% Trims Engine Data to fit inside rpm bounds (0, redline)
redline_I = find(engine_torque_data(:,1)>redline);
engine_torque_data = engine_torque_data(1:redline_I(1),:);

% Data conversion factors
rpm_conv = (2*pi()*Rt)/60; % rpm to m/s
Nm_conv = 1/(Rt); % Nm to N

% Converts engine torque data to acceleration(m/s2) / velocity(m/s) 
engine_force_vel = [engine_torque_data(:,1)*rpm_conv, engine_torque_data(:,2)*Nm_conv];

% Declare annonymous accel sim function
vehicle_param = @(FDr) [vehicle_parameters(1), vehicle_parameters(2), FDr];
time_calc = @(FDr) AccelSim(engine_force_vel, engine_gear_ratio, vehicle_param(FDr), coeff_inputs, 75, false);

% Use annonymous function to find fastest final drive ratio
min_final_drive = fminsearch(time_calc, 3.5);

% Iterate through different final drive ratios and calculate drag times 
times = [];
i_range = linspace(1,10,100);
for i = i_range
    times = [times, time_calc(i)];
end 
figure();
    plot(i_range, times)
    xlabel("Final Drive Ratio")
    ylabel("75m Drag Time")

% Result Readout
disp("Optimum Final Drive")
disp(min_final_drive)

disp("75m drag time:")
disp(AccelSim(engine_force_vel, engine_gear_ratio, vehicle_parameters, coeff_inputs, 75, true))

disp("Final Drive Ratio:")
disp(final_drive_ratio)