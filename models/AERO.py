from .SystemModel import SystemModel

class AERO(SystemModel):
    def __init__(self):
        self.Cl = 1.77
        self.Cd = 0.8
        self.CoP = 0.48