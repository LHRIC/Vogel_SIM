function Fy = mfeval_wrapper(X)
load("./utilities/Lateral_Tire-Model_Optim_Params.mat")
Fy = MF52_Fy_fcn(X, OptimParameterSet);