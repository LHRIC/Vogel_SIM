from Vehicle import Vehicle
import numpy as np
import matplotlib.pyplot as plt

class Engine:

    def __init__(self, AERO, DYN, PTN, trajectory) -> None:
        self._vehicle = Vehicle(AERO=AERO, DYN=DYN, PTN=PTN, trajectory_path=trajectory)

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
        
        

    def sweep(self, num_steps, **kwargs) -> None:
        if(len(kwargs) > 1):
            print("WARN: Coupled parameter sweeps are not yet supported")
            return
        else:
            sweep_param = list(kwargs.keys())[0]
            sweep_class = None
            if getattr(self._vehicle.AERO, sweep_param, None) is not None:
                sweep_class = self._vehicle.AERO
            elif getattr(self._vehicle.DYN, sweep_param, None) is not None:
                sweep_class = self._vehicle.DYN
            elif getattr(self._vehicle.PTN, sweep_param, None) is not None:
                sweep_class = self._vehicle.PTN
            else:
                print(f'ERROR: Parameter "{sweep_param}" not found')
                return
            sweep_range = kwargs[sweep_param]
            sweep_range = np.linspace(sweep_range[0], sweep_range[1], num=num_steps)
            times = []
            for p in sweep_range:
                setattr(sweep_class, sweep_param, p)
                self._vehicle.GGV.generate()
                laptime = self._vehicle.simulate()
                times.append(laptime)
                self._vehicle.reset_ggv()

            fig, ax = plt.subplots()
            ax.plot(sweep_range, times, "o-")
            plt.show()
        
