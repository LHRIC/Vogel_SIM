clear all
global FZ0 LFZO LCX LMUX LEX LKX  LHX LVX LCY LMUY LEY LKY LHY LVY ...
       LGAY LTR LRES LGAZ LXAL LYKA LVYKA LS LSGKP  LSGAL LGYR KY

load("2022_Lateral_Coeff_B2356run4.mat")
load("A1654run21_MF52_Fy_GV12.mat")
FZ0 = 150
PCY1= 2.1504
PDY1= 2.4372
PDY2=-0.1189
PDY3= 14.0751
PEY1= 1.4040
PEY2= -0.5023
PEY3= 0
PEY4= 0
PEY5= 0
PKY1= -40.7740
PKY2= 135.6443
PKY3= 0.7045
PKY4= 151.1476
PKY5= 1.2708e+03
PKY6= 3.3459
PKY7= 1.5696
PHY1= -1.8571e-04
PHY2= 4.6741e-05
PHY3 = 0
PVY1= 0.0059
PVY2= -0.0299
PVY3= 0.6819
PVY4= 0.3205
PPY1= -0.0086
PPY2= 0.7544
PPY3= -0.3261
PPY4= -0.9817
PPY5= -0.9331
RBY1= 4.9000
RBY2= 2.200

A = [PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 PKY6 PKY7 PHY1 PHY2 PHY3 PVY1 PVY2 PVY3 PVY4 PPY1 PPY2 PPY3 PPY4 PPY5 RBY1 RBY2] 
Fz = 167;
alpha = -13.4129:.01:13.4129;
IA = 0;
for i = 1:length(alpha)
X = [alpha(i) Fz IA];
Fy(i) = MF52_Fy_fcn(A, X);
end

% ALPHA  =  alpha*pi/180;
% Fz     =  abs(X(:,1));
% GAMMA  =  X(:,2)*pi/180;
% 
% GAMMAy = GAMMA .* LGAY; %31 (%48 lgay=lg
% Fz0PR  = FZ0  .*  LFZO; %15,  NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
% DFz    = (Fz-Fz0PR) ./ Fz0PR; %14,  (%30)
% 
% % Setting initial parameters
% PCy1    = A(1);
% PDy1    = A(2);
% PDy2    = A(3);
% PDy3    = A(4);
% PEy1    = A(5);
% PEy2    = A(6);
% PEy3    = A(7);
% PEy4    = A(8);
% PKy1    = A(9);
% PKy2    = A(10);
% PKy3    = A(11);
% PHy1    = A(12);
% PHy2    = A(13);
% PHy3    = A(14);
% PVy1    = A(15);
% PVy2    = A(16);
% PVy3    = A(17);
% PVy4    = A(18);
% 
% 
% 
% SHy     = (PHy1+PHy2 .* DFz) .* LHY + PHy3 .* GAMMAy; %38,  (%55)
% ALPHAy  = ALPHA+SHy;  %30 (%47)
% Cy      = PCy1 .* LCY;  %32 (%49)
% MUy     = (PDy1+PDy2 .* DFz) .* (1.0-PDy3 .* GAMMAy.^2) .* LMUY; %34 (%51)
% Dy      = MUy .* Fz; %33 (%50)
% KY      = PKy1 .* FZ0 .* sin(2.0 .* atan(Fz ./ (PKy2 .* FZ0 .* LFZO))) .* (1.0-PKy3 .* abs(GAMMAy)) .* LFZO .* LKY; %36 (%53)
% By      = KY ./ (Cy .* Dy);  %37 (%54)
% Ey      = (PEy1+PEy2 .* DFz) .* (1.0-(PEy3+PEy4 .* GAMMAy) .* sign(ALPHAy)) .* LEY; %35 (%52)
% SVy     = Fz .* ((PVy1+PVy2 .* DFz) .* LVY+(PVy3+PVy4 .* DFz) .* GAMMAy) .* LMUY; %39 (%56)
% Fy0     = Dy .* sin(Cy .* atan(By .* ALPHAy-Ey .* (By .* ALPHAy-atan(By .* ALPHAy))))+SVy; %29 (%46)
% Fy      = Fy0; %28
figure
plot(alpha, Fy)
hold on