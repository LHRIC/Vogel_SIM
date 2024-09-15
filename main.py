from SimEngine import Engine
import models

def main():


    simulator = Engine(track_id="./tracks/2019_IC_michigan_endurance",steps=1000,track_length=2000,GGV_detail=20)
    
    # simulator.sweep(num_steps=20, torsional_rigidity=(850, 2000), xlabel="Torsional Rigidity (Nm/deg)")
    # simulator.sweep(num_steps=10, total_weight=(51.89, 52.79), xlabel="Total Weight (lbs.)")
    # simulator.sweep(num_steps=20, cg_height=(12.0, 12.2), xlabel="Cg Height (in.)")
    #simulator.sweep(num_steps = 15, Cd=(0.5, 1.2), xlabel="CDA")
    # simulator.sweep(num_steps = 20, trackwidth_f=(45, 70), xlabel = "Track Width")
    # simulator.sweep(num_steps = 30, xlabel="Final Drive", final_drive=(2, 5))
    # simulator.sweep(num_steps = 20, camber_gain_r=(-0.2, 0.1), xlabel="Camber Gain (deg/mm)")
    # simulator.sweep(num_steps=10, Cl=(2,5), xlabel = "Cla")
    #simulator.sweep(num_steps=20, LLTD=(0.4,0.7), xlabel = "LLTD")
    #simulator.test_ggv() 
    simulator.single_run(run_mode="ENDURANCE", plot=True)
      # After loop ends

if __name__ == "__main__":
    main()