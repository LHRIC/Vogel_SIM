function Fy = MF52_Fy_fcn(A,X)
% This function completes the MF5.2 Fitting of tire data provided by the
% Tire Testing Consortium
% This function is for Lateral Cases Only

global FZ0 LFZO LCX LMUX LEX LKX  LHX LVX LCY LMUY LEY LKY LHY LVY ...
       LGAY LTR LRES LGAZ LXAL LYKA LVYKA LS LSGKP  LSGAL LGYR KY 
    
ALPHA  =  X(:,1)*pi/180;
Fz     =  (X(:,2));
GAMMA  =  X(:,3)*pi/180;
dpi = 0;


gamma_star = GAMMA .* LGAY; %31 (%48 lgay=lg
Fz0_prime  = abs(FZ0  .*  LFZO); %15,  NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
dfz    = (Fz-Fz0_prime) ./ Fz0_prime; %14,  (%30)

% Setting initial parameters
PCY1 = A(1);
PDY1 = A(2);
PDY2 = A(3);
PDY3 = A(4);
PEY1 = A(5);
PEY2 = A(6);
PEY3 = A(7);
PEY4 = A(8);
PEY5 = A(9);
PKY1 = A(10);
PKY2 = A(11);
PKY3 = A(12);
PKY4 = A(13);
PKY5 = A(14);
PKY6 = A(15);
PKY7 = A(16);
PHY1 = A(17);
PHY2 = A(18);
PHY3 = A(19);
PVY1 = A(20);
PVY2 = A(21);
PVY3 = A(22);
PVY4 = A(23);
PPY1 = A(24);
PPY2 = A(25);
PPY3 = A(26);
PPY4 = A(27);
PPY5 = A(28);
RBY1 = A(29);
RBY2 = A(30);


Kya = PKY1.*Fz0_prime.*(1 + PPY1.*dpi).*(1 - PKY3.*abs(gamma_star)).*sin(PKY4.*atan((Fz./Fz0_prime)./((PKY2+PKY5.*gamma_star.^2).*(1+PPY2.*dpi)))).*LKY; % (= ByCyDy = dFyo./dalphay at alphay = 0) (if gamma =0: =Kya0 = CFa) (PKY4=2)(4.E25)
SVyg = Fz.*(PVY3 + PVY4.*dfz).*gamma_star; % (4.E28)
Kyg0 = Fz.*(PKY6 + PKY7 .*dfz).*(1 + PPY5.*dpi); % (=dFyo./dgamma at alpha = gamma = 0) (= CFgamma) (4.E30)
signKya = sign(Kya);
signKya(signKya == 0) = 1; % If [Kya = 0] then [sign(0) = 0]. This is done to avoid [num / 0 = NaN] in Eqn 4.E27
SHy = (PHY1 + PHY2.*dfz).* LHY + ((Kyg0 .*gamma_star - SVyg)./Kya); % (4.E27) [sign(Kya) term explained on page 177]
SVy = Fz.*(PVY1 + PVY2.*dfz).*LVY + SVyg; % (4.E29)
alphay = ALPHA + SHy; % (4.E20)
Cy = PCY1.*LCY; % (> 0) (4.E21)
muy = (PDY1 + PDY2 .* dfz).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1 - PDY3.*gamma_star.^2); % (4.E23)
Dy = muy.*Fz; % (4.E22)
signAlphaY = sign(alphay);
signAlphaY(signAlphaY == 0) = 1;
Ey = (PEY1 + PEY2.*dfz).*(1 + PEY5.*gamma_star.^2 - (PEY3 + PEY4.*gamma_star).*signAlphaY).*LEY; % (<=1)(4.E24)
if(any(Ey > 1))            
    Ey(Ey > 1) = 1;
end % if Ey > 1
signDy = sign(Dy);
signDy(signDy == 0) = 1; % If [Dy = 0] then [sign(0) = 0]. This is done to avoid [Kya / 0 = NaN] in Eqn 4.E26
By = Kya./(Cy.*Dy); % (4.E26) [sign(Dy) term explained on page 177]
Fy = Dy .* sin(Cy.*atan(By.*alphay-Ey.*(By.*alphay - atan(By.*alphay))))+ SVy; % (4.E19)


% SHy     = (PHy1+PHy2 .* DFz) .* LHY + PHy3 .* GAMMAy; %38,  (%55)
% ALPHAy  = ALPHA+SHy;  %30 (%47)
% Cy      = PCy1 .* LCY;  %32 (%49)
% MUy     = (PDy1+PDy2 .* DFz) .* (1.0-PDy3 .* GAMMAy.^2) .* LMUY; %34 (%51)
% Dy      = MUy .* Fz; %33 (%50)
% Kya = PKY1.*FZ0.*(1 + PPY1.*dpi).*(1 - PKY3.*abs(GAMMAy)).*sin(PKY4.*atan((Fz./FZ0)./((PKY2+PKY5.*GAMMAy.^2).*(1+PPY2.*dpi)))).*zeta3.*LKYBy      = KY ./ (Cy .* Dy);  %37 (%54)
% % NOTE, PER SVEN @TNO: "SIGN(ALPHAY)"IS CORRECT AS IN DOCUMENTATION & BELOW; IT'S NOT SUPPOSED TO BE "SIGN(GAMMAY)"
% Ey      = (PEy1+PEy2 .* DFz) .* (1.0-(PEy3+PEy4 .* GAMMAy) .* sign(ALPHAy)) .* LEY; %35 (%52)
% % NOTE: LVY MULTIPLIES ONLY PVY1&2 IN DOCUMENTATION; ORIG VERSION MULT ALL TERMS
% SVy     = Fz .* ((PVy1+PVy2 .* DFz) .* LVY+(PVy3+PVy4 .* DFz) .* GAMMAy) .* LMUY; %39 (%56)
% Fy0     = Dy .* sin(Cy .* atan(By .* ALPHAy-Ey .* (By .* ALPHAy-atan(By .* ALPHAy))))+SVy; %29 (%46)
% Fy      = Fy0; %28
% Fy = Fy
