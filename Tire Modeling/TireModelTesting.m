clear all
global FZ0 LFZO LCX LMUX LEX LKX  LHX LVX LCY LMUY LEY LKY LHY LVY ...
       LGAY LTR LRES LGAZ LXAL LYKA LVYKA LS LSGKP  LSGAL LGYR KY OptimParameterSet

FZ0 = 800
%% Lateral Tire Model
load("Lateral_Tire-Model_Optim_Params.mat")

% alpha = -.23:.01:.23;
% IA = 0;
% Fzspace = linspace(800,3000,100);
% for k =1:length(Fzspace)
%     Fz = Fzspace(1,k);
%     for i = 1:length(alpha)
%     X = [alpha(i) Fz IA];
%     Fy(i) = MF52_Fy_fcn(X);
%     
%     end
%     plot(alpha, Fy)
%     hold on
% end

%% Longitudinal Tire model
% PCX1= 1.323844878928801
% PDX1= 2.144821849495536
% PDX2= -.1
% PDX3= .001167915841767
% PEX1= -3.473271185688184
% PEX2= -4.089223400248820
% PEX3= .374047785439375
% PEX4= -1.564832874142583e-18
% PKX1= 25
% PKX2= .1
% PKX3= -.784633274177115
% PHX1= -.004218240866128
% PHX2= -.003129880120497
% PVX1= -.016657065489241
% PVX2= -.10000000000
% PPX1= -.601337395748281
% PPX2= .014446194764119
% PPX3= -.262573151301394
% PPX4= .026601005872547
% RBX1= 0
% RBX2= 0
% RBX3= 0
% RCX1= 0
% REX1= 0
% REX2= 0
% RHX1= 0
load("Blakes_Longitdudinal_Coeff.mat")


% Bx = [PCX1 PDX1 PDX2 PDX3 PEX1 PEX2 PEX3 PEX4 PKX1 PKX2 PKX3 PHX1 PHX2 PVX1 PVX2 PPX1 PPX2 PPX3 PPX4 RBX1 RBX2 RBX3 RCX1 REX1 REX2 RHX1] 
Fz = 200;
SL = -1:.01:1;
IA = 0;
X = [Fz IA];

Fzspace = linspace(800,2000,100);
for k =1:length(Fzspace)
    Fz = Fzspace(1,k);
    X = [Fz IA];
    Fx = MF52_Fx_fcn(lCoeff,X,SL); 
    plot(SL, Fx)
    hold on
end
