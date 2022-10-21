function Fx = MF52_Fx_fcn(Bx,X,SL)
% This function completes the MF6.2 Fitting of tire data provided by the
% Tire Testing Consortium
% This function is Longitudinal cases only
% This tire model is METRIC

global FZ0
    
Fz = abs(X(:,1));

GAMMAx  =  X(:,2);


GAMMAx = GAMMAx;
Fz0PR  = abs(FZ0);
DFz    = (Fz-Fz0PR) ./ Fz0PR;

% Setting initial parameters
PCx1    = Bx(1);
PDx1    = Bx(2);
PDx2    = Bx(3);
% PDx3    = .001167915841767
PDx3    = Bx(4);
PEx1    = Bx(5);
PEx2    = Bx(6);
PEx3    = Bx(7);
PEx4    = Bx(8);
% PKx1    = 25
PKx1    = Bx(9);
% PKx2    = .1
PKx2    = Bx(10);
% PKx3    = -.784633274177115
PKx3    = Bx(11);
PHx1    = Bx(12);
PHx2    = Bx(13);
PVx1    = Bx(14);
PVx2    = Bx(15);
PPx1    = Bx(16);
PPx2    = Bx(17);
PPx3    = Bx(18);
PPx4    = Bx(19);
% RBx1    = 0;
% RBx2    = 0;
% RBx3    = 0;
% RCx1    = 0;
% REx1    = 0;
% REx2    = 0;
% RHx1    = 0;
RBx1    = Bx(20);
RBx2    = Bx(21);
RBx3    = Bx(22);
RCx1    = Bx(23);
REx1    = Bx(24);
REx2    = Bx(25);
RHx1    = Bx(26);

% Influence of camber
SHx = RHx1;
Ex = REx1+REx2*DFz;
Cx = RCx1;
GAMMAstar = sin(GAMMAx);
Bxa = (RBx1+RBx3.*GAMMAstar)*cos(atan(RBx2*SL));
ALPHA = 0; % Slip angle is defined as zero since this function is only used for straight line, can be revised later to include complete combined slip 
Gxao = cos(Cx*atan(Bxa*SHx-Ex*(Bxa*SHx-atan(Bxa*SHx))));
Gxa = cos(ALPHA)./Gxao; % Gxa should actually have a bunch of stuff in the cos(x), see above comment

% Pure longitudinal 
SHx = (PHx1 + PHx2 * DFz);
Cx = PCx1;
MUx = (PDx1 + PDx2 * DFz);
Dx = MUx * Fz;
Kx = Fz * (PKx1 + PKx2 * DFz) * exp(PKx3 * DFz);
Bxa = Kx / (Cx * Dx);
SLx = SL + SHx;
Ex = (PEx1 + PEx2 * DFz + PEx3 * (DFz)^2) * (1 - PEx4*sign(SLx));
SVx = Fz * (PVx1 + PVx2 * DFz);
Fx0 = Dx * sin(Cx * atan(Bxa .* SLx - Ex .* (Bxa .* SLx - atan(Bxa .* SLx)))) + SVx;

% Combined Slip
Fx = Fx0.*Gxa;