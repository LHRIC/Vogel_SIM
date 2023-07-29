load("Accel time sensitivity to COP and Weight Distribution.mat")
load("Endurance time sensitivity to COP and Weight Distribution.mat")

%% Endurance time sensitivity to COP and Weight Distribution.mat
XData = c.XData;
YData = c.YData;
ZData = c.ZData;
[row, column] = find(ZData>84);
XData(row,column) = NaN;
YData(row,column) = NaN;
ZData(row,column) = NaN;
EnduranceCOPandWD(XData,YData,ZData)

%% Accel time sensitivity to COP and Weight Distribution.mat
AccelCOPandWD(d.XData,d.YData,d.ZData)

%% Endurance and Accel Time sensitivity to LLTD and Weight Distribution.mat
load("Endurance and Accel Time sensitivity to LLTD and Weight Distribution.mat")

XData = c.XData;
YData = c.YData;
ZData = c.ZData;
% [row, column] = find(ZData>84);
% XData(row,column) = NaN;
% YData(row,column) = NaN;
% ZData(row,column) = NaN;
EnduranceLLTDandWD(XData,YData,ZData)

AccelLLTDandWD(d.XData,d.YData,d.ZData)

%% Accel Sensitivity to Weight
load("WeightSensitivity.mat")
AccelWeight(Weight,Accel_time)

%% Endurance Sensitivity to Weight
EnduranceWeight(Weight,Endurance_time)

%% Accel Sensitivity to Cg height
load("CgSensitivity.mat")
AccelCg(Cg,Accel_time)
%% Enduracne Sensitivity to Cg height
EnduranceCg(Cg,Endurance_time)
%% Accel Sensitivity to Weight Distribution (includes COP follower)
load('WDistribution.mat')
AccelWD(weightDistribution,Accel_time)
%% Endurance Sensitivty to Weight Distribution (includes COP follower) 
EnduranceWD(weightDistribution,Endurance_time)
%% Accel Sensitivity to TrackWidth
load('TrackWidth.mat')
AccelTW(tw.*39.37,Accel_time)
%% Endurance Sensitivty to TrackWidth
EnduranceTW(tw.*39.37,Endurance_time)
%% Accel Sensitivity to TrackWidth
load('Friction.mat')
AccelFriction(tw.*39.37,Accel_time)
%% Endurance Sensitivty to TrackWidth
EnduranceFriction(tw.*39.37,Endurance_time)
