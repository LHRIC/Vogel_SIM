from SimEngine import Engine
import models

def main():


    simulator = Engine(trajectory="./trajectory/23_michigan_autox_ft.csv", is_closed=False)
    
    #simulator.sweep(num_steps=20, torsional_rigidity=(850, 2000), xlabel="Torsional Rigidity (Nm/deg)")
    simulator.sweep(num_steps=10, total_weight=(650, 660), xlabel="Total Weight (lbs.)")
    #simulator.sweep(num_steps=20, cg_height=(12.0, 12.2), xlabel="Cg Height (in.)")
    #simulator.sweep(num_steps = 15, Cd=(0.5, 1.2), xlabel="CDA")
    #simulator.sweep(num_steps = 30, trackwidth_f=(1.2, 1.35))
    #simulator.sweep(num_steps = 30, xlabel="Final Drive", final_drive=(2, 5))
    #simulator.sweep(num_steps = 20, camber_gain_p=(1, 2), xlabel="Camber Gain (p)")
    #simulator.single_run(run_mode="SKIDPAD", plot=True)
    #simulator.test_ggv()

if __name__ == "__main__":
    main()