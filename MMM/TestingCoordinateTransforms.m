clear
body_angle = deg2rad(5);
steered_angle = deg2rad(6);
x = [1.5 0.05 -0.2];

AY_NTB = x(1);
yaw_rate = x(2);
AX_NTB = x(3);
AY_IMF = AY_NTB*cos(body_angle) + AX_NTB*sin(body_angle); % (g's)
AX_IMF = AX_NTB*cos(body_angle) - AY_NTB*sin(body_angle); % (g's)

Fy_fin_TIR = 50; % tire
Fy_fin_IMF = Fy_fin_TIR*cos(steered_angle);
Fx_fin_IMF_tan = Fy_fin_IMF*tan(steered_angle);
Fx_fin_IMF_sin = Fy_fin_TIR*sin(steered_angle);
