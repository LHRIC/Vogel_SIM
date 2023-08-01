import math
import matlab.engine

class MF52:
    def __init__(self):
        self.__matlab_engine = matlab.engine.start_matlab()
        # This is called from the root path for some reason.
        # possibly called in the location of the file that created
        # an instance of the MF52 class.
        s = self.__matlab_engine.genpath('./utilities')
        self.__matlab_engine.addpath(s, nargout=0)
        self.FZ0 = 800  # Nominal tire load: newtons
        self.lCoeff = [1.32384487892880, 2.14482184949554, -0.100000000000000, 0.00116791584176700, -3.47327118568818, -4.08922340024882, 0.374047785439375, -1.56483287414258e-18, 25, 0.100000000000000, -
                          0.784633274177115, -0.00421824086612800, -0.00312988012049700, -0.0166570654892410, -0.100000000000000, -0.601337395748281, 0.0144461947641190, -0.262573151301394, 0.0266010058725470, 0, 0, 0, 0, 0, 0, 0]
        
    def Fx(self, Fz, GammaX, SL):
        Fz = abs(Fz)
        Fz0PR = abs(self.FZ0)
        DFz = (Fz - Fz0PR) / Fz0PR

        # Setting initial parameters
        PCx1 = self.lCoeff[0]
        PDx1 = self.lCoeff[1]
        PDx2 = self.lCoeff[2]
        PDx3 = self.lCoeff[3]
        PEx1 = self.lCoeff[4]
        PEx2 = self.lCoeff[5]
        PEx3 = self.lCoeff[6]
        PEx4 = self.lCoeff[7]
        PKx1 = self.lCoeff[8]
        PKx2 = self.lCoeff[9]
        PKx3 = self.lCoeff[10]
        PHx1 = self.lCoeff[11]
        PHx2 = self.lCoeff[12]
        PVx1 = self.lCoeff[13]
        PVx2 = self.lCoeff[14]
        PPx1 = self.lCoeff[15]
        PPx2 = self.lCoeff[16]
        PPx3 = self.lCoeff[17]
        PPx4 = self.lCoeff[18]
        RBx1 = self.lCoeff[19]
        RBx2 = self.lCoeff[20]
        RBx3 = self.lCoeff[21]
        RCx1 = self.lCoeff[22]
        REx1 = self.lCoeff[23]
        REx2 = self.lCoeff[24]
        RHx1 = self.lCoeff[25]

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

    def Fy(self, ALPHA, Fz, GAMMA):
        print("Staring Matlab Engine")
        X = matlab.double([ALPHA, Fz, GAMMA])
        Fy = self.__matlab_engine.mfeval_wrapper(X)
        return Fy

    @staticmethod
    def test():
        print('yay')