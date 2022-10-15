clear variables
clc 

load("Output_Files\tqSens_925.mat")
time = [LapSimOutput.laptime];
load("Output_Files\tqSens_950.mat")
time = [time LapSimOutput.laptime];
load("Output_Files\tqSens_975.mat")
time = [time LapSimOutput.laptime];
load("Output_Files\tqSens_1000.mat")
time = [time LapSimOutput.laptime];
load("Output_Files\tqSens_1025.mat")
time = [time LapSimOutput.laptime];
load("Output_Files\tqSens_1050.mat")
time = [time LapSimOutput.laptime];
load("Output_Files\tqSens_1075.mat")
time = [time LapSimOutput.laptime];

%%
reltime = 100*(time-time(4))/time(4);
tqMod_vals = [.925 .95 .975 1 1.025 1.05 1.075];

tiledlayout(1,2)
hold on

nexttile
plot(tqMod_vals,time)
title('Laptime Sensitivity to Scaling Torque Curve')
ylabel('Laptime (s)')

nexttile
plot(tqMod_vals,reltime)
xlabel('Torque Curve Multiplier')
ylabel('Laptime (% change)')




