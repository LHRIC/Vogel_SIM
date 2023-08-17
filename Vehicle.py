import models
import ggv.GGV as GGV
import trajectory.Trajectory as Trajectory
from numpy.polynomial import Polynomial
import numpy as np
import math
import matplotlib.pyplot as plt

import matlab

class Vehicle:
    def __init__(self, trajectory_path, AERO, DYN, PTN, mesh_resolution=10):
        self.AERO = AERO
        self.DYN = DYN
        self.PTN = PTN        

        '''Total Reduction'''
        self.gear_tot = self.PTN.gear_ratios[-1] * self.PTN.final_drive * self.PTN.primary_reduction
        '''Max Achievable Velocity, assuming RPM never exceeds shiftpoint'''
        self.v_max = self.PTN.shiftpoint / (self.gear_tot/self.DYN.tire_radius*60/(2 * math.pi))
        self.GGV = GGV.GGV(self.AERO, self.DYN, self.PTN, self.gear_tot, self.v_max)
        
        self.trajectory = Trajectory.Trajectory(trajectory_path, self.GGV.radii_range[0], self.GGV.radii_range[-1])
        self._interval = mesh_resolution

        self._mesh_size = (self.trajectory.num_points) * self._interval
        self.count = np.zeros(self._mesh_size)

        self.time = np.zeros(self._mesh_size)

        self.x = np.zeros(self._mesh_size)
        self.y = np.zeros(self._mesh_size)

        '''Vehicle State Attributes'''
        self.dist = np.zeros(self._mesh_size)
        self.dist_f = np.zeros(self._mesh_size)
        self.dist_r = np.zeros(self._mesh_size)

        self.turn_dir = np.zeros(self._mesh_size)

        self.velocity = np.zeros(self._mesh_size)
        self.velocity_f = np.zeros(self._mesh_size)
        self.velocity_r = np.zeros(self._mesh_size)

        self.gear = np.zeros(self._mesh_size)

        self.ax = np.zeros(self._mesh_size)
        self.ax_f = np.zeros(self._mesh_size)
        self.ax_r = np.zeros(self._mesh_size)

        self.ay = np.zeros(self._mesh_size)
        self.ay_f = np.zeros(self._mesh_size)
        self.ay_r = np.zeros(self._mesh_size)
    
        def __del__(self):
            self._matlab_engine.quit()


    def reset_ggv(self):
        self.GGV = GGV.GGV(self.AERO, self.DYN, self.PTN, self.gear_tot, self.v_max, self._matlab_engine)   

    def simulate(self):
        vel = 20 * 0.3048; # m/s
        self.simulate_forwards(vel)
        self.simulate_reverse()
        self.simulate_forwards(self.velocity_r[0])

        for i in self.count:
            i = int(i)
            self.dist[i] = self.dist_f[i]
            if self.velocity_f[i] < self.velocity_r[i]:
                self.velocity[i] = self.velocity_f[i]
                self.ax[i] = self.ax_f[i]
                self.ay[i] = self.ay_f[i]
            else:
                self.velocity[i] = self.velocity_r[i]
                self.ax[i] = -1 * self.ax_r[i]
                self.ay[i] = self.ay_r[i]

        '''Determine turn handedness and remove outliers'''
        self.ay[self.ay > self.GGV.lateral_capability.evaluate(self.v_max + 1)] = self.GGV.lateral_capability.evaluate(self.v_max + 1)
        self.ay_f[self.ay_f > self.GGV.lateral_capability.evaluate(self.v_max + 1)] = self.GGV.lateral_capability.evaluate(self.v_max + 1)
        self.ay_r[self.ay_r > self.GGV.lateral_capability.evaluate(self.v_max + 1)] = self.GGV.lateral_capability.evaluate(self.v_max + 1)
        
        self.ay = np.multiply(self.ay, self.turn_dir)
        self.ay_f = np.multiply(self.ay_f, self.turn_dir)
        self.ay_r = np.multiply(self.ay_r, self.turn_dir)
        
        self.ax_r = np.multiply(self.ax_r, -1)
        
        '''
        ax.plot(self.dist_f, comb)
        plt.show()
        plt.scatter(self.x, self.y, c=comb)
        plt.colorbar()
        plt.show()
        '''

        return max(self.time)  
    
    def simulate_forwards(self, starting_v):
        vel = starting_v
        gear = 1
        required_gear = gear
        time_shifting = 0
        is_shifting = False
        count = 0
        distance= 0
        time = 0
        for point_idx in range(self.trajectory.num_points):
            x_1 = self.trajectory.points[0][point_idx]
            y_1 = self.trajectory.points[1][point_idx]
            x_2 = self.trajectory.points[0][(point_idx + 1) % self.trajectory.num_points]
            y_2 = self.trajectory.points[1][(point_idx + 1) % self.trajectory.num_points]

            '''Distance of trajectory interval in meters'''
            dist = math.sqrt((x_1-x_2)**2 + (y_2-y_1)**2)

            r = self.trajectory.radii[point_idx]

            '''Max Achievable Cornering Velocity through this segment'''
            v_max = min(self.v_max, self.GGV.cornering_capability.evaluate(r))

            for idx, v in enumerate(self.GGV.velocity_range):
                if(v > vel):
                    required_gear = self.GGV.expected_gears[idx]
                    break
            
            is_shifting = required_gear != gear

            AX_cap = self.GGV.accel_capability.evaluate(vel)
            AY_cap = self.GGV.lateral_capability.evaluate(vel)
            AY_actual = vel ** 2 / (r * 9.81)

            delta_d = dist / self._interval

            for j in range(self._interval):
                self.x[count] = x_1 + (x_2 - x_1) * j/self._interval
                self.y[count] = y_1 + (y_2 - y_1) * j/self._interval

                self.ay_f[count] = AY_actual

                self.turn_dir[count] = np.sign(self.trajectory._curvature[point_idx])

                self.count[count] = count
                self.velocity_f[count] = vel
                self.gear[count] = gear
                self.dist_f[count] =  distance + delta_d * j

                dt = delta_d / vel
                self.time[count] = time + dt * j
                if is_shifting and vel < v_max:
                    # Currently shifting, do not accelerate
                    dt = delta_d / vel
                    time_shifting += dt
                    self.ax_f[count] = 0
                elif vel < v_max:
                    # Take a slice of the GGV using AX_cap and AY_cap as verticies
                    # the actual lateral acceleration will define the remaining longitudinal acceleration.
                    ax_potential = AX_cap*math.sqrt(1-(min(AY_cap,AY_actual)/AY_cap)**2)
                    self.ax_f[count] = ax_potential

                    # Solve the 1D kinematic equation for time
                    p = Polynomial([-delta_d, vel, 0.5*9.81*ax_potential])
                    tt = p.roots()
                    dt = max(tt)

                    #Solve for change in velocity
                    dv = 9.81 * ax_potential * dt

                    #Clamp new velocity to v_max
                    vel = min(v_max, vel + dv)

                    for i in range(len(self.GGV.expected_gears)):
                        if(self.GGV.expected_gears[i] > vel):
                            required_gear = self.GGV.expected_gears[i]
                    
                    is_shifting = required_gear > gear
                else:
                    # Vehicle is neither shifting or accelerating, is at vmax
                    vel = v_max
                    dt = delta_d / vel
                    self.ax_f[count] = 0

                

                if time_shifting >= self.PTN.shift_time:
                    is_shifting = False
                    time_shifting = 0
                    gear = required_gear
                
                count += 1
                time += dt
            
            distance += dist
        '''
        c = []
        for i in range(self._mesh_size + 1):
            if i % self._interval == 0:
                c.append(self.gear[i])
            
        plt.scatter(self.trajectory.points[0], self.trajectory.points[1], c=c)
        plt.colorbar()
        plt.show()
        '''

    def simulate_reverse(self):
        vel = self.velocity_f[-1]
        distance = 0
        count = int(self.count[-1])
        for point_idx in range(self.trajectory.num_points - 1, -1, -1):
            x_1 = self.trajectory.points[0][point_idx]
            y_1 = self.trajectory.points[1][point_idx]
            x_2 = self.trajectory.points[0][(point_idx - 1) % self.trajectory.num_points]
            y_2 = self.trajectory.points[1][(point_idx - 1) % self.trajectory.num_points]

            '''Distance of trajectory interval in meters'''
            dist = math.sqrt((x_1-x_2)**2 + (y_2-y_1)**2)

            r = self.trajectory.radii[point_idx]

            '''Max Achievable Cornering Velocity through this segment'''
            v_max = min(self.v_max, self.GGV.cornering_capability.evaluate(r))

            AX_cap = self.GGV.accel_capability.evaluate(vel)
            AY_cap = self.GGV.lateral_capability.evaluate(vel)
            AY_actual = vel ** 2 / (r * 9.81)

            delta_d = dist/self._interval
            for j in range(self._interval):
                self.velocity_r[count] = vel
                self.dist_r[count] =  distance + delta_d * j

                self.ay_r[count] = AY_actual
                if vel < v_max:
                    AX_actual = AX_cap*(1-(min(AY_cap,AY_actual)/AY_cap)**2)
                    self.ax_r[count] = AX_actual
                    p = Polynomial([-delta_d, vel, 0.5*9.81*AX_actual])
                    tt = p.roots()
                    dt = max(tt)
                    dv = 9.81 * AX_actual * dt
                    dv_max = v_max - vel
                    vel += min(dv, dv_max)
                else:
                    vel = v_max
                    self.ax_r[count] = 0

                count -= 1
            distance += dist

