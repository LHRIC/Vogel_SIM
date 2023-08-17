from Vehicle import Vehicle
import numpy as np
import matplotlib.pyplot as plt
from joblib import Parallel, delayed
from multiprocessing import Manager, Pool, Queue, cpu_count
import os
import scipy.interpolate as interp

class Engine:

    def __init__(self, AERO, DYN, PTN, trajectory) -> None:
        self._trajectory_path = trajectory
        self._AERO = AERO
        self._DYN = DYN
        self._PTN = PTN
        
        self._sweep_param = None
        self._sweep_class = None

    def single_run(self):
        vehicle = Vehicle(AERO=self._AERO, DYN=self._DYN, PTN=self._PTN, trajectory_path=self._trajectory_path)
        vehicle.GGV.generate()
        laptime = vehicle.simulate()
        print(laptime)
        
        Ax_f = vehicle.ax_f
        Ay_f = vehicle.ay_f

        V_f = vehicle.velocity_f

        ploty1, plotz1 = np.meshgrid(np.linspace(np.min(Ay_f), np.max(Ay_f), 60),
                                      np.linspace(np.min(V_f), np.max(V_f), 60))
        
        plotx1 = interp.griddata((Ay_f, V_f), Ax_f, (ploty1, plotz1),
                                 method='linear', fill_value=0.0)
        
        Ax_r = vehicle.ax_r
        Ay_r = vehicle.ay_r

        V_r = vehicle.velocity_r
        # Griddata
        ploty2, plotz2, = np.meshgrid(np.linspace(np.min(Ay_r), np.max(Ay_r), 60),
                                      np.linspace(np.min(V_r), np.max(V_r), 60))
        plotx2 = interp.griddata((Ay_r, V_r), Ax_r, (ploty2, plotz2),
                                 method='linear', fill_value=0.0)

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        surf1 = ax.plot_surface(plotx1, ploty1, plotz1,
                                cstride=1, rstride=1, cmap='coolwarm',
                                edgecolor='black', linewidth=0.2,
                                antialiased=True)
        
        surf2 = ax.plot_surface(plotx2, ploty2, plotz2,
                                cstride=1, rstride=1, cmap='coolwarm',
                                edgecolor='black', linewidth=0.2,
                                antialiased=True)
        
         # Add a color bar which maps values to colors.
        fig.colorbar(surf1)

        plt.title("GGV")
        plt.xlabel('ax [g]')
        plt.ylabel('ay [g]')
        ax.set_zlabel('velocity [m/s]')
        plt.show()
        '''

        

        fig, ax = plt.subplots()
        ax.axis('equal')
        plt.scatter(vehicle.ay, vehicle.ax)
        plt.show()

        plt.scatter(vehicle.x, vehicle.y, c=vehicle.velocity)
        plt.show()

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot(vehicle.ay, vehicle.ax, vehicle.velocity, "o", markersize=1)
        ax.set_xlabel('Lateral Accel. (g)')
        ax.set_ylabel('Longitudinal Accel (g)')
        ax.set_zlabel('Velocity (m/s)')
        plt.xlim([-1.5, 1.5])
        plt.ylim([-1.5, 1.5])
        plt.show()
        '''

    def test_ggv(self):
        vehicle = Vehicle(AERO=self._AERO, DYN=self._DYN, PTN=self._PTN, trajectory_path=self._trajectory_path)
        vehicle.GGV.generate()
        vehicle.GGV.plot()
        
        

    def sweep(self, num_steps, **kwargs) -> None:
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

            num_processes = cpu_count() * 2
            with Pool(num_processes) as p:
                times = p.map(self.compute_task, sweep_range)
                        

            fig, ax = plt.subplots()
            ax.plot(sweep_range, list(times), "o-")
            plt.show()
        
    def compute_task(self, p):
        
        '''
        print(str(os.getpid()) + ": " + q.get())
        '''
        print("Simulating: " + self._sweep_param + " = " + str(p))
        vehicle = Vehicle(AERO=self._AERO, DYN=self._DYN, PTN=self._PTN, trajectory_path=self._trajectory_path)
        sweep_model = getattr(vehicle, self._sweep_class)
        setattr(sweep_model, self._sweep_param, p)
        sweep_model.unit_conv()
        
        vehicle.GGV.generate()
        laptime = vehicle.simulate()
        return laptime

