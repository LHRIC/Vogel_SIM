from utilities import MF52
import setups
import numpy as np

class TireState():
    # TODO: setups thing here
    def __init__(self, params: setups.Goose):
        self._MF52 = MF52()
        self.params = params

        #Slip angle
        self.alpha = 0

        #Longitudinal Slip
        self.kappa = 0

        #Inclination angle
        self.epsilon = 0
        
        #Normal Load
        self.Fz = 0

        self.Fx = 0 #N
        self.Fy = 0 #N
    
    '''Returns the unscaled longitudinal force in the tire frame'''
    def eval_Fx(self):
        fx_range = []
        for sl in np.linspace(0, 0.2, 100):
            fx_range.append(self._MF52.Fx(self.Fz, sl, self.epsilon) * self.params.friction_scaling_x)
        self.Fx = max(fx_range)
    
    '''Returns the unscaled lateral force in the tire frame'''
    def eval_Fy(self):
        self.Fy = self._MF52.Fy(self.Fz, self.alpha, self.epsilon) * self.params.friction_scaling_y

    