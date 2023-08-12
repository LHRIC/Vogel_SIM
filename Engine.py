from Vehicle import Vehicle
import numpy as np
import matplotlib.pyplot as plt

class Engine:

    def __init__(self, AERO, DYN, PTN, trajectory) -> None:
        self._vehicle = Vehicle(AERO=AERO, DYN=DYN, PTN=PTN, trajectory_path=trajectory)

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
        
