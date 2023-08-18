import math
import matlab.engine
import numpy as np
import pickle
from functools import cache
import matplotlib.pyplot as plt

from scipy.interpolate import RegularGridInterpolator

class MF52:
    def __init__(self):
        self.FZ0 = 800  # Nominal tire load: newtons
        self.long_coeff = [1.32384487892880, 2.14482184949554, -0.100000000000000, 0.00116791584176700, -3.47327118568818, -4.08922340024882, 0.374047785439375, -1.56483287414258e-18, 25, 0.100000000000000, -
                          0.784633274177115, -0.00421824086612800, -0.00312988012049700, -0.0166570654892410, -0.100000000000000, -0.601337395748281, 0.0144461947641190, -0.262573151301394, 0.0266010058725470, 0, 0, 0, 0, 0, 0, 0]
        self.lat_coeff = [0.349, -0.00115, 8.760, 730.300, 1745.322, 0.0139, -0.000277, 1.02025435, 0, 0, 0, 0, 0, 0, 0, 0.00362, -0.0143, -0.0116]
        self.c_Fx = None
        self.c_Fy = None

        with open("./utilities/FxInterpolator.pickle", "rb") as Fx_in:
            self.c_Fx = pickle.load(Fx_in)
            Fx_in.close()
        with open("./utilities/FyInterpolator.pickle", "rb") as Fy_in:
            self.c_Fy = pickle.load(Fy_in)
            Fy_in.close()

        self.c_Fx.bounds_error = False
        self.c_Fy.bounds_error = False

    def Fx_interp(self, Fz, GammaX, SL):
        Fx = self.c_Fx((Fz, GammaX, SL))
        if(math.isnan(Fx)):
            raise RuntimeError(f"Requested parameter ({Fz}, {GammaX}, {SL}) outside of Fx tire model bounds") 
        else:
            return Fx

    def Fx(self, Fz, GammaX, SL):
        Fz = abs(Fz)
        Fz0PR = abs(self.FZ0)
        DFz = (Fz - Fz0PR) / Fz0PR

        # Setting initial parameters
        PCx1 = self.long_coeff[0]
        PDx1 = self.long_coeff[1]
        PDx2 = self.long_coeff[2]
        PDx3 = self.long_coeff[3]
        PEx1 = self.long_coeff[4]
        PEx2 = self.long_coeff[5]
        PEx3 = self.long_coeff[6]
        PEx4 = self.long_coeff[7]
        PKx1 = self.long_coeff[8]
        PKx2 = self.long_coeff[9]
        PKx3 = self.long_coeff[10]
        PHx1 = self.long_coeff[11]
        PHx2 = self.long_coeff[12]
        PVx1 = self.long_coeff[13]
        PVx2 = self.long_coeff[14]
        PPx1 = self.long_coeff[15]
        PPx2 = self.long_coeff[16]
        PPx3 = self.long_coeff[17]
        PPx4 = self.long_coeff[18]
        RBx1 = self.long_coeff[19]
        RBx2 = self.long_coeff[20]
        RBx3 = self.long_coeff[21]
        RCx1 = self.long_coeff[22]
        REx1 = self.long_coeff[23]
        REx2 = self.long_coeff[24]
        RHx1 = self.long_coeff[25]

        SHx = RHx1
        Ex = REx1+REx2*DFz
        Cx = RCx1
        GAMMAstar = math.sin(GammaX)
        Bxa = (RBx1+RBx3*GAMMAstar)*math.cos(math.atan(RBx2*SL))
        ALPHA = 0  # Slip angle is defined as zero since this function is only used for straight line, can be revised later to include complete combined slip
        Gxao = math.cos(Cx*math.atan(Bxa*SHx-Ex*(Bxa*SHx-math.atan(Bxa*SHx))))
        # % Gxa should actually have a bunch of stuff in the cos(x), see above comment
        Gxa = math.cos(ALPHA)/Gxao

        # Pure longitudinal
        SHx = (PHx1 + PHx2 * DFz)
        Cx = PCx1
        MUx = (PDx1 + PDx2 * DFz)
        Dx = MUx * Fz
        Kx = Fz * (PKx1 + PKx2 * DFz) * math.exp(PKx3 * DFz)
        Bxa = Kx / (Cx * Dx)
        SLx = SL + SHx
        Ex = (PEx1 + PEx2 * DFz + PEx3 * (DFz) ** 2) * (1 - PEx4*(SLx/abs(SLx)))
        SVx = Fz * (PVx1 + PVx2 * DFz)
        Fx0 = Dx * math.sin(Cx * math.atan(Bxa * SLx - Ex *
                            (Bxa * SLx - math.atan(Bxa * SLx)))) + SVx

        # Combined Slip
        Fx = Fx0*Gxa

        return Fx

    def Fy_old(self, ALPHA, Fz, GAMMA, engine=None):
        X = matlab.double([ALPHA, Fz, GAMMA])
        if engine is None:
            Fy = self.__matlab_engine.mfeval_wrapper(X)
        else:
            Fy = engine.mfeval_wrapper(X)
        return Fy
    
    def Fy(self, SL, Fz, IA):
        SL = math.degrees(SL)
        IA = math.degrees(IA)

        [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17] = self.lat_coeff
        C = a0
        D = Fz * (a1 * Fz + a2) * (1 - a15 * IA ** 2)

        BCD = a3 * math.sin(math.atan(Fz / a4) * 2) * (1 - a5 * abs(IA))
        B = BCD / (C * D)

        H = a8 * Fz + a9 + a10 * IA
        E = (a6 * Fz + a7) * (1 - (a16 * IA + a17) * math.copysign(1, SL + H))
        V = a11 * Fz + a12 + (a13 * Fz + a14) * IA * Fz
        Bx1 = B * (SL + H)
        
        return (D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V) * -1
    

if __name__ == '__main__':
    tm = MF52()
    '''
    wr = np.linspace(-2000, 0, 2000)
    IA_r = np.linspace(-5, 5, 50)
    sl = np.linspace(-0.1, 1, 100)


    engine = matlab.engine.start_matlab()
    tm = MF52()
    
    Fx_arr = np.zeros((2000, 50, 100))

    for i in range(len(wr)):
        print(wr[i])
        for j in range(len(IA_r)):
            for k in range(len(sl)):
                Fx_arr[i, j, k] = tm.Fx_old(wr[i], IA_r[j], sl[k])
    
    fn = RegularGridInterpolator((wr, IA_r, sl), Fx_arr)

    with open('./utilities/FxInterpolator.pickle', 'wb') as pickle_file:
        pickle.dump(fn, pickle_file, pickle.HIGHEST_PROTOCOL)
    
    
    
    a_r = np.linspace(-0.5, 0.5, 50)
    wr = np.linspace(-300, 1500, 200)
    IA_r = np.linspace(-0.01, 0.2, 50)

    engine = matlab.engine.start_matlab()
    s = engine.genpath('./utilities')
    engine.addpath(s, nargout=0)
    tm = MF52()

    Fy_arr = np.zeros((50, 200, 50))

    
    for i in range(len(a_r)):
        print(a_r[i])
        for j in range(len(wr)):
            for k in range(len(IA_r)):
                Fy_arr[i, j, k] = tm.Fy_old(a_r[i], wr[j], IA_r[k], engine=engine)
    
    fn = RegularGridInterpolator((a_r, wr, IA_r), Fy_arr)

    with open('./utilities/FyInterpolator.pickle', 'wb') as pickle_file:
        pickle.dump(fn, pickle_file, pickle.HIGHEST_PROTOCOL)
    '''