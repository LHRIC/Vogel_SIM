from Vehicle import Vehicle
import numpy as np
import matplotlib.pyplot as plt

class Engine:

    def __init__(self, AERO, DYN, PTN, trajectory) -> None:
        self._vehicle = Vehicle(AERO=AERO, DYN=DYN, PTN=PTN, trajectory_path=trajectory)

    def single_run(self):
        self._vehicle.GGV.generate()
        laptime = self._vehicle.simulate()
        
        '''
        plt.scatter(self._vehicle.x, self._vehicle.y, c=self._vehicle.ax)
        plt.colorbar()

        plt.show()
        '''
        fig, ax = plt.subplots()
        ax.axis('equal')
        plt.scatter(self._vehicle.ay, self._vehicle.ax)
        plt.show()

        plt.plot(self._vehicle.dist, self._vehicle.velocity, markersize=1)
        plt.show()

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot(self._vehicle.ay, self._vehicle.ax, self._vehicle.velocity, "o", markersize=1)
        ax.set_xlabel('Lateral Accel. (g)')
        ax.set_ylabel('Longitudinal Accel (g)')
        ax.set_zlabel('Velocity (m/s)')
        plt.xlim([-1.5, 1.5])
        plt.ylim([-1.5, 1.5])
        plt.show()
        
        

    def sweep(self, lb, ub, step_size) -> None:
        sweep_range = np.linspace(lb, ub, num=step_size)
        times = []
        for p in sweep_range:
            self._vehicle.GGV.DYN.total_weight = p
            self._vehicle.GGV.generate()
            laptime = self._vehicle.simulate()
            times.append(laptime)
            self._vehicle.reset_ggv()

        fig, ax = plt.subplots()
        ax.plot(sweep_range, times, "o-")
        plt.show()
        
