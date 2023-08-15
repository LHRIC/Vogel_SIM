from Vehicle import Vehicle
import numpy as np
import matplotlib.pyplot as plt
from joblib import Parallel, delayed
from multiprocessing import Manager, Pool, Queue 
import os

class Engine:

    def __init__(self, AERO, DYN, PTN, trajectory) -> None:
        self._trajectory_path = trajectory
        self._AERO = AERO
        self._DYN = DYN
        self._PTN = PTN

        #print("Starting Matlab Engine")
        
        self._sweep_param = None
        self._sweep_class = None
        #self._vehicle = Vehicle(AERO=AERO, DYN=DYN, PTN=PTN, trajectory_path=trajectory)

    def single_run(self):
        self._vehicle.GGV.generate()
        laptime = self._vehicle.simulate()
        
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
        
        

    def sweep(self, num_steps, multiprocessing=False, **kwargs) -> None:
        if(len(kwargs) > 1):
            print("WARN: Coupled parameter sweeps are not yet supported")
            return
        else:
            self._sweep_param = list(kwargs.keys())[0]
            if getattr(self._AERO, self._sweep_param, None) is not None:
                self._sweep_class = "AERO"
            elif getattr(self._DYN, self._sweep_param, None) is not None:
                self._sweep_class = "DYN"
            elif getattr(self._PTN, self._sweep_param, None) is not None:
                self._sweep_class = "PTN"
            else:
                print(f'ERROR: Parameter "{self._sweep_param}" not found')
                return
            sweep_range = kwargs[self._sweep_param]
            sweep_range = np.linspace(sweep_range[0], sweep_range[1], num=num_steps)
            times = []

            with Pool(10) as p:
                times = p.map(self.sr, sweep_range)
                        

            fig, ax = plt.subplots()
            ax.plot(sweep_range, list(times), "o-")
            plt.show()
        
    def sr(self, p):
        
        '''
        print(str(os.getpid()) + ": " + q.get())
        '''
        print("Simulating")
        vehicle = Vehicle(AERO=self._AERO, DYN=self._DYN, PTN=self._PTN, trajectory_path=self._trajectory_path)
        sweep_model = getattr(vehicle, self._sweep_class)
        setattr(sweep_model, self._sweep_param, p)
        vehicle.GGV.generate()
        laptime = vehicle.simulate()
        return laptime

