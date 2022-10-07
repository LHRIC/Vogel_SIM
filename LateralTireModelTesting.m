global FZ0

FZ0 = -150
load("2022_Lateral_Coeff_B2356run4.mat")

Fz = -179.847;
alpha = -6:0.001:6;
IA = 0;
X = [Fz IA];

F_x = MF52_Fy_fcn(A, X, alpha);

plot(alpha, F_x)
hold on
