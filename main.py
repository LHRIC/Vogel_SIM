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
    #simulator.sweep(num_steps=30, total_weight=(300, 600))
    #simulator.sweep(num_steps = 15, cg_height=(10, 15))
    #simulator.sweep(num_steps = 30, trackwidth_f=(1.2, 1.35))
    simulator.single_run()
    #simulator.test_ggv()

if __name__ == "__main__":
    main()