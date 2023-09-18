from Engine import Engine
import models

def main():


    simulator = Engine(trajectory="./trajectory/17_lincoln_endurance_track_highres.xls")
    
    simulator.sweep(num_steps=20, trackwidth_f=(47.224, 53.15))
    #simulator.sweep(num_steps = 15, cg_height=(10, 15))
    #simulator.sweep(num_steps = 30, trackwidth_f=(1.2, 1.35))
    #simulator.sweep(num_steps = 30, run_mode="ACCEL", final_drive=(1, 5))
    #simulator.single_run(run_mode="ENDURANCE", plot=True)
    #simulator.test_ggv()

if __name__ == "__main__":
    main()