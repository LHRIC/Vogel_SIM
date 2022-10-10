clear variables
close all
clc

track = readtable('circle_tracktestpoints.xlsx');
t = 1:height(track);
path_points = [track.X, track.Y]';

x = linspace(1,t(end-1),1000);
ppv = pchip(t,path_points);
vehicle_path = ppval(ppv,x);
vehicle_path_EN = vehicle_path;
Length = arclength(vehicle_path(1,:),vehicle_path(2,:));

plot(vehicle_path(1),vehicle_path(2))