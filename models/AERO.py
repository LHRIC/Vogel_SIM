from .SystemModel import SystemModel

class AERO():
    def __init__(self, overrides={}):
        self.Cl = 1.77
        self.Cd = 0.8
        self.CoP = 0.48

        for key in overrides.keys():
            val = overrides[key]

            try:
                setattr(self, key, val)
            except:
                exit("Ahhhh")
        self.compute_deriv_params()
        self.convert_units()
    
    def compute_deriv_params(self):
        pass

    def convert_units(self):
        pass