import models

class Vehicle:
    def __init__(self):
        self.PTN = models.PTN()
        self.DYN = models.DYN()
        self.AERO = models.AERO()
        
        #TODO: gearTot line 53
        #TODO: VMAX: line 54


if __name__ == "__main__":
    v = Vehicle()
    print(v.AERO.Cd)