import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

class Trajectory:
    '''Racing Line Generation'''
    def __init__(self, file, is_closed, r_min, r_max) -> None:
        self.num_points = 0
        self.points = [[], []]
        self.is_closed = is_closed
        self._curvature = None
        self.radii = None
        self._dist = []

        self.load(file)

        #self.calc_curvature(self.points[0], self.points[1])
        #self._curvature[self._curvature == 0] = 1e-12
        #self.radii = np.divide(1, np.abs(self._curvature))
        self.num_points = len(self.radii)
        # Clamp the trajectory turn radii to the GGV limits
        self.radii[self.radii > r_max] = r_max
        self.radii[self.radii < r_min] = r_min


    def load(self, file):
        '''For now, load Trajectory from csv'''
        if file.split(".")[-1] in ["xls", "xslx"]:
            df = pd.read_excel(file)
        else:
            df = pd.read_csv(file)
        
        #normalize ft to meters
        self.points[0] = np.multiply(df["X"].to_numpy(), 0.3048)
        self.points[1] = np.multiply(df["Y"].to_numpy(), 0.3048)

        #TODO dont hardcode, fix later, time crunch
        curv_df = pd.read_csv("./trajectory/trackmaps/23_michigan_autox_curvature_m.csv")
        self._curvature = curv_df["curvature"].to_numpy()
        radii_df = pd.read_csv("./trajectory/trackmaps/23_michigan_autox_radii_m.csv")
        self.radii = radii_df["radii"].to_numpy()

    def filter_outliers(self):
        dist = []
        for i in range((len(self.points[0]) - 1)):
            x_1 = self.points[0][i]
            x_2 = self.points[0][i+1]
            y_1 = self.points[1][i]
            y_2 = self.points[1][i+1]

            dist.append(math.sqrt((x_2 - x_1)**2 + (y_2 - y_1)**2))
        
        dist = np.array(dist)

        d = np.abs(dist - np.median(dist))
        mdev = np.median(d)
        s = d/mdev if mdev else np.zeros(len(d))

        dist = np.insert(dist, 0, 0, axis=0)
        s = np.insert(s, 0, 0, axis=0)

        tol = 6
        self.points[0] = self.points[0][s<tol]
        self.points[1] = self.points[1][s<tol]
        self._dist = dist[s<tol]

        self.num_points = len(self.points[0])

if __name__ == "__main__":
    import itertools
    t = Trajectory("./trajectory/trackmaps/23_michigan_autox_ft.csv", False, 4.5, 36)
    #plt.scatter(t.points[0], t.points[1], c=abs(t._curvature))
    x = []
    y = []
    curv = []

    for i in range(len(t.points[0])):
        x.append(t.points[0][i])
        y.append(t.points[1][i])
        curv.append(i)

    print(t._dist)
    plt.scatter(x, y, c=t.radii)
    plt.axis('equal')
    plt.colorbar()
    plt.show()