from Vehicle import Vehicle
import matplotlib.pyplot as plt
import math
from numpy.polynomial import Polynomial
import numpy as np

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
    vehicle.simulate_reverse()
    vehicle.simulate_forwards(vehicle.velocity_r[0])
    print(vehicle.dist_f)
    print(vehicle.velocity_f)

    fig, ax = plt.subplots()
    comb = np.zeros(len(vehicle.velocity_r))
    for i in vehicle.count:
        i = int(i)
        m = min(vehicle.velocity_f[i], vehicle.velocity_r[i])
        comb[i] = m


    ax.plot(vehicle.dist_f, comb)
    plt.show()
    plt.scatter(vehicle.x, vehicle.y, c=comb)
    plt.colorbar()
    plt.show()




if __name__ == "__main__":
    main()