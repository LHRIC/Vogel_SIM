from Vehicle import Vehicle
import matplotlib.pyplot as plt
import math
from numpy.polynomial import Polynomial

def main():
    vehicle = Vehicle("./trajectory/17_lincoln_endurance_track_highres.xls")
    vehicle.GGV.generate()
    GGV = vehicle.GGV

    
    '''
    plt.scatter(t._trajectory[0], t._trajectory[1], c=t._radii)
    plt.colorbar()
    plt.show()
    '''
    vel = 20 * 0.3048; # m/s
    vehicle.simulate_forwards(vel)




if __name__ == "__main__":
    main()