clear variables
close all
clc

%% load data
load('Plots/RPM_vs_time_data.mat')

tireD = 7.6*2*.0254;
finalDrive = 37/11;
primaryReduction = 76/36;
gear = [33/12, 32/16, 30/18, 26/18, 30/23, 29/24];

gear = primaryReduction * finalDrive * gear;

wheelRPM = LapSimOutput.velocity*60/(tireD*pi);

RPM = wheelRPM.*gear(LapSimOutput.gear_counter);

figure
% plot(LapSimOutput.time_elapsed,RPM)
plot(LapSimOutput.time_elapsed,LapSimOutput.gear_counter)

T = table(LapSimOutput.time_elapsed', RPM', 'VariableNames', ["Time", "RPM"]);
writetable(T,'Plots/RPM_vs_time.xls','Sheet',1)
% excel = readtable('RPM_vs_time.xls')

