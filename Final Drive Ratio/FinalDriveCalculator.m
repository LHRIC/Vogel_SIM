%% SETUP
close all;
clear all;
clc;

g = 9.81;

%% Car Specs

% Vehicle Specs
Rt = 0.2032; % wheel radius
effective_mass = 294.835;
redline = 12000; % desired maximum rpm
f_max = 10000;
drag_coeff = 1.2923; % N/(m/s)^2

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
final_drive_ratio = 3.5;

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

% Declare annonymous functions
ResistiveForce_anon = @(v) ResistiveForce(v,drag_coeff);
time_calc = @(FDr) AccelSim(engine_force_vel, f_max, ResistiveForce_anon, effective_mass, engine_gear_ratio, FDr, 75, false);

% DONT QUESTION THIS, I'm too dumb to debug why fminsearch is not working and this is my solution
times = [];
i_range = linspace(1,10,50);
for i = i_range
    times = [times, time_calc(i)];
end 
figure();
    plot(i_range, times)
    xlabel("Final Drive Ratio")
    ylabel("75m Drag Time")

times = [];
i_range = linspace(1,10,500);
for i = i_range
    times = [times, time_calc(i)];
end 
figure();
    plot(i_range, times)
    xlabel("Final Drive Ratio")
    ylabel("75m Drag Time")

disp("75m drag time:")
disp(AccelSim(engine_force_vel, f_max, ResistiveForce_anon, effective_mass, engine_gear_ratio, final_drive_ratio, 75, true))

disp("Final Drive Ratio:")
disp(final_drive_ratio)