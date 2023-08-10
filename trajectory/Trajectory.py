import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class Trajectory:
    '''Racing Line Generation'''
    def __init__(self, file, r_min, r_max) -> None:
        self.num_points = 0
        self.points = [[], []]
        self._curvature = None
        self.radii = None

        self.load(file)
        self.calc_curvature(self.points[0], self.points[1])
        self._curvature[self._curvature == 0] = 1e-6
        self.radii = np.divide(1, self._curvature)

        # Clamp the trajectory turn radii to the GGV limits
        self.radii[self.radii > r_max] = r_max
        self.radii[self.radii < r_min] = r_min


    def load(self, file):
        '''For now, load Trajectory from csv'''
        df = pd.read_excel(file)
        #normalize to meters
        self.points[0] = np.divide(df["X"].to_numpy(), 1000)
        self.points[1] = np.divide(df["Y"].to_numpy(), 1000)
        self.num_points = len(self.points[0])

    def calc_curvature(self, X, Y):
        x_t = np.gradient(X)
        y_t = np.gradient(Y)

        vel = np.array([ [x_t[i], y_t[i]] for i in range(x_t.size)])
        speed = np.sqrt(x_t * x_t + y_t * y_t)

        xx_t = np.gradient(x_t)
        yy_t = np.gradient(y_t)

        self._curvature = np.abs(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t)**1.5
        
        


if __name__ == "__main__":
    t = Trajectory("./track/17_lincoln_endurance_track_highres.xls", 4.5, 36)
    plt.scatter(t.trajectory[0], t.trajectory[1], c=t.radii)
    plt.colorbar()
    plt.show()