close all;
clear all;
clc;



load("Accel_For_Remy.mat")
load("Grip_For_Remy.mat")

range = linspace(4.5,30);
hold off
figure
hold on
plot(range,polyval(grip,range))
hold on
grid on
grid minor
hold on
fnplt(accel)
grid on
grid minor