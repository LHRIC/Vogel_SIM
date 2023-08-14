from Engine import Engine
from trajectory import Trajectory
import models
import numpy as np
import math

def main():
    aero_model = models.AERO()
    dyn_model = models.DYN()
    ptn_model = models.PTN()

    simulator = Engine(AERO=aero_model, DYN=dyn_model, PTN=ptn_model, trajectory="./trajectory/17_lincoln_endurance_track_highres.xls")
    simulator.single_run()


if __name__ == "__main__":
    main()