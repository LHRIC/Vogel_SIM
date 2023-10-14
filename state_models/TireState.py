from utilities import MF52

class TireState():
    def __init__(self):
        self._MF52 = MF52()

        #Slip angle
        self.alpha = 0

        #Longitudinal Slip
        self.kappa = 0

        #Inclination angle
        self.epsilon = 0
        
        #Normal Load
        self.Fz = 0

        #Longitudinal slip
        self.kappa = 0
    
    '''Returns the unscaled longitudinal force in the tire frame'''
    def Fx(self):
        return self._MF52.Fx(self.Fz, self.epsilon, self.kappa)
    
    '''Returns the unscaled lateral force in the tire frame'''
    def Fy(self):
        return self._MF52.Fy(self.alpha, self.Fz, self.epsilon)

    