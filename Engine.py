from Vehicle import Vehicle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from multiprocessing import Pool, cpu_count
import scipy.interpolate as interp
import models
import setups
import csv
import math

#plt.style.use('seaborn-v0_8-white')

class Engine:

    def __init__(self, trajectory, is_closed) -> None:
        self._trajectory_path = trajectory
        self._is_closed = is_closed
        
        self._sweep_params = []
        self._sweep_classes = []
        self._sweep_bounds = []

        self._run_mode = "ENDURANCE"

    def single_run(self, run_mode="ENDURANCE", plot=False):
        params = setups.Goose()

        vehicle = Vehicle(params=params, trajectory_path=self._trajectory_path, is_closed=self._is_closed)
        vehicle.GGV.generate()
        laptime = 0

        self._run_mode = run_mode.upper()

        if(self._run_mode == "ENDURANCE"):
            laptime = vehicle.simulate_endurance()
            laptime *= 10
            Tmax = 1973.419
            Tmin = 1360.978
            score = 250 * ((Tmax / laptime) - 1)/((Tmax / Tmin) - 1) + 25
            print("Laptime (10 laps):", laptime)
            print(f"Score: {score}")
        elif(self._run_mode == "ACCEL"):
            laptime = vehicle.simulate_accel()

            Tmax = 4.174
            Tmin = 6.262
            score = 95.5 * ((Tmax / laptime) - 1)/((Tmax / Tmin) - 1) + 4.5
            print("Accel Time:", laptime)
            print(f"Score: {score}")
        elif(self._run_mode == "SKIDPAD"):
            path_radius = 18.25/2
            skidpad_vel = vehicle.GGV.cornering_capability.evaluate(path_radius)
            
            Tmax = 6.135
            Tmin = 4.908
            laptime = path_radius*2*math.pi/skidpad_vel + 0.125 # Assume one cone because the lateral accel calc is a bit overzelous :(
            score = 71.5 * ((Tmax / laptime)**2 - 1)/((Tmax / Tmin)**2 - 1) + 3.5

            print("Laptime:", laptime)
            print(f"Score: {score}")
        elif(self._run_mode == "AUTOX"):
            laptime = vehicle.simulate_endurance()

            Tmax = 66.534
            Tmin = 45.886
            score = 118.5 * ((Tmax / laptime) - 1)/((Tmax / Tmin) - 1) + 6.5
            print("Laptime:", laptime)
            print(f"Score: {score}")
        else:
            print(f"Invalid run mode: {self._run_mode}, please select either ENDURANCE or ACCEL.")
            return
        
        if plot:
            if(self._run_mode == "ENDURANCE" \
               or self._run_mode == "AUTOX"):
                Ax_f = np.zeros(len(vehicle.ax_f))
                Ay_f = np.zeros(len(vehicle.ay_f))

                V_f = np.zeros(len(vehicle.velocity_f))


                for i in range(len(vehicle.ax_f)):
                    if(vehicle.is_shifting[i] != 1):
                        Ax_f[i]  = vehicle.ax_f[i]
                        Ay_f[i]  = vehicle.ay_f[i]
                        V_f[i]  = vehicle.velocity_f[i]
                '''

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

                fig1 = plt.figure()
                ax1 = fig1.add_subplot(projection='3d')
                surf1 = ax1.plot_surface(plotx1, ploty1, plotz1,
                                        cstride=1, rstride=1, cmap='coolwarm',
                                        edgecolor='black', linewidth=0.2,
                                        antialiased=True)
                
                surf2 = ax1.plot_surface(plotx2, ploty2, plotz2,
                                        cstride=1, rstride=1, cmap='coolwarm',
                                        edgecolor='black', linewidth=0.2,
                                        antialiased=True)
                
                # Add a color bar which maps values to colors.
                fig1.colorbar(surf1)

                plt.title("GGV")
                plt.xlabel('Ax [g]')
                plt.ylabel('Ay [g]')
                ax1.set_zlabel('Velocity [m/s]')

                '''

                fig2, ax2 = plt.subplots()
                ax2.axis('equal')
                ax2.plot(vehicle.ay, vehicle.ax, "o")

                fig3, ax3 = plt.subplots()
                sc = ax3.scatter(vehicle.x, vehicle.y, c=vehicle.velocity)
                ax3.axis("equal")
                cbar = plt.colorbar(sc)
                cbar.set_label("Velocity (m/s)")

                fig6, ax6 = plt.subplots()
                grip_lim_x = []
                grip_lim_y = []
                vel_cb = []
                for i in range(len(vehicle.velocity)):
                    if vehicle.velocity[i] < 13:
                        grip_lim_x.append(vehicle.x[i])
                        grip_lim_y.append(vehicle.y[i])
                        vel_cb.append(vehicle.velocity[i])


                sc = ax6.scatter(grip_lim_x, grip_lim_y, c=vel_cb)
                cbar = plt.colorbar(sc)

                print("%% Grip Limited: ", len(vel_cb) / len(vehicle.velocity))


                fig4 = plt.figure()
                ax4 = fig4.add_subplot(projection='3d')
                ax4.plot(vehicle.ay, vehicle.ax, vehicle.velocity, "o", markersize=1)
                ax4.set_xlabel('Lateral Accel. (g)')
                ax4.set_ylabel('Longitudinal Accel (g)')
                ax4.set_zlabel('Velocity (m/s)')
                plt.xlim([-2.5, 2.5])
                plt.ylim([-2.5, 2.5])

                fig5, ax5 = plt.subplots()
                #ax5.plot(np.arange(0, len(vehicle.velocity)), vehicle.velocity, "o--", markersize=2)
                ax5.plot(vehicle.time, vehicle.velocity_f)
                ax5.plot(vehicle.time, vehicle.velocity_r, "r")
                ax5_1 = ax5.twinx()
                ax5_1.scatter(vehicle.time, vehicle.is_shifting)
                plt.xlabel("Distance (m)")
                plt.ylabel("Time (s)")

                fig, ax6 = plt.subplots()
                ax6.plot(vehicle.time, vehicle.velocity)
                plt.xlabel("Time (s)")
                plt.ylabel("Velocity (m/s)")

                with open("./CharlesVvT.csv", "w") as outfile:
                    csv_writer = csv.writer(outfile, delimiter=',')
                    csv_writer.writerow(["Time (s)", "Velocity (m/s)"])
                    for i in range(len(vehicle.time)):
                        csv_writer.writerow([vehicle.time[i], vehicle.velocity[i]])

                print("Avg vel:", sum(vehicle.velocity)/len(vehicle.velocity))
                plt.show(block=False)
                plt.pause(0.001) # Pause for interval seconds.
                input("hit[enter] to end.")
                plt.close('all') # all open plots are correctly closed after each run
            elif(self._run_mode == "ACCEL"):
                fig, ax = plt.subplots()
                ax.scatter(vehicle.x, vehicle.y, c=vehicle.velocity)

                fig, ax = plt.subplots()
                ax.plot(vehicle.dist, vehicle.ax)
                plt.show(block=False)
                plt.pause(0.001) # Pause for interval seconds.
                input("hit[enter] to end.")
                plt.close('all') # all open plots are correctly closed after each run

    def sweep(self, num_steps, run_mode="ENDURANCE", xlabel="", **kwargs) -> None:
        self._run_mode = run_mode.upper()

        self._sweep_params = list(kwargs.keys())

        if(len(kwargs) > 1):
            print("ERROR: Coupled parameter sweeps are not yet supported")
            return

        for param in self._sweep_params:
            self._sweep_bounds.append(kwargs[param])

        sweep_range = np.linspace(self._sweep_bounds[0][0], self._sweep_bounds[0][1], num=num_steps)
        
        payloads = []
        times = []

        for param_val in sweep_range:
            vehicle_params = setups.VehicleSetup(overrides={self._sweep_params[0]: param_val})


            payloads.append({
                'PARAMS': vehicle_params,
                'COUNT': param_val
            })


        
        num_processes = cpu_count() * 2
        with Pool(num_processes) as p:
            times = p.map(self.compute_task_authoritative, payloads)
        
        print("Sensitivity: ", (max(times) - min(times)) / (max(sweep_range) - min(sweep_range)))
        
        fig, ax = plt.subplots()
        ax.plot(sweep_range, list(times), "o-")
        plt.xlabel(xlabel)
        plt.ylabel("Laptime (s)")
        plt.show()


    def compute_task_authoritative(self, p):
        params = p["PARAMS"]
        count = p["COUNT"]

        vehicle = Vehicle(params=params, trajectory_path=self._trajectory_path)
        
        vehicle.GGV.generate()

        laptime = 0
        if(self._run_mode == "ENDURANCE"):
            laptime = vehicle.simulate_endurance()
        elif(self._run_mode == "ACCEL"):
            laptime = vehicle.simulate_accel()

        #fig2, ax2 = plt.subplots()
        #ax2.axis('equal')
        #ax2.plot(vehicle.ay, vehicle.ax, "o")
        #save_location = './sweeps/' + str("%.2f" % round(count,2)).replace(".", "_") + ".jpg"
        #plt.savefig(save_location)
        
        return laptime

        
    
        
    def compute_task(self, p):
        vehicle = Vehicle(AERO=self._AERO, DYN=self._DYN, PTN=self._PTN, trajectory_path=self._trajectory_path)
        if(type(p) == tuple):
            # Coupled parameter sweep
            for i in range(len(p)):
                sweep_model = getattr(vehicle, self._sweep_classes[i])
                setattr(sweep_model, self._sweep_params[i], p[i])
                sweep_model.unit_conv()
        else:
            # Single param sweep
            sweep_model = getattr(vehicle, self._sweep_classes[0])
            setattr(sweep_model, self._sweep_params[0], p)
            sweep_model.unit_conv()
        
        vehicle.GGV.generate()

        laptime = 0
        if(self._run_mode == "ENDURANCE"):
            laptime = vehicle.simulate_endurance()
        elif(self._run_mode == "ACCEL"):
            laptime = vehicle.simulate_accel()
        return laptime

