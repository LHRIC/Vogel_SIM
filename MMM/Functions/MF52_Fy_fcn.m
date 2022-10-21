function Fy = MF52_Fy_fcn(X)
% Tire Testing Consortium
% This function is for Lateral Cases Only

global OptimParameterSet


ALPHA  =  X(:,1);
Fz     =  X(:,2);
GAMMA  =  X(:,3);


% inputsMF = [Fz kappa alpha gamma phit Vx P* omega*], where
% Fz     = normal load on the tyre  [N]
% kappa  = longitudinal slip        [dimensionless, -1: locked wheel]
% alpha  = side slip angle          [rad]
% gamma  = inclination angle        [rad]
% phit   = turn slip                [1/m]
% Vx     = forward velocity         [m/s]
% P*     = pressure                 [Pa]
% omega* = rotational speed         [rad/s]

MFinput = [Fz 0 ALPHA GAMMA 0 16 82737.1];   %build MFinput
Fyeval = mfeval(OptimParameterSet,MFinput,121);
Fy = (Fyeval(:,2));
end