N = 696.9;
load("longitudinal tire parameters.mat")

B = [OptimParameterSet.PCX1 OptimParameterSet.PDX1 OptimParameterSet.PDX2 OptimParameterSet.PDX3 OptimParameterSet.PEX1 OptimParameterSet.PEX2 OptimParameterSet.PEX3 OptimParameterSet.PEX4 OptimParameterSet.PKX1 OptimParameterSet.PKX2 OptimParameterSet.PKX3 OptimParameterSet.PHX1 OptimParameterSet.PHX2 OptimParameterSet.PVX1 OptimParameterSet.PVX2 OptimParameterSet.PPX1 OptimParameterSet.PPX2 OptimParameterSet.PPX3 OptimParameterSet.PPX4 OptimParameterSet.RBX1 OptimParameterSet.RBX2 OptimParameterSet.RBX3 OptimParameterSet.RCX1 OptimParameterSet.REX1 OptimParameterSet.REX2 OptimParameterSet.RHX1 ];
SL = -0.5:0.001:0.5;

Fz = -150;
IA = 0;
X = [Fz IA];

F_x = MF52_Fx_fcn(B, X,SL);

plot(SL, F_x)
hold on
