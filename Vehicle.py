import models
import ggv.GGV as GGV
import trajectory.Trajectory as Trajectory
from numpy.polynomial import Polynomial
import numpy as np
import math
import matplotlib.pyplot as plt

import matlab

class Vehicle:
    def __init__(self, trajectory_path):
        self.AERO = models.AERO()
        self.DYN = models.DYN()
        self.PTN = models.PTN()

        '''Total Reduction'''
        self.gear_tot = self.PTN.gear_ratios[-1] * self.PTN.final_drive * self.PTN.primary_reduction
        '''Max Achievable Velocity, assuming RPM never exceeds shiftpoint'''
        self.v_max = self.PTN.shiftpoint / (self.gear_tot/self.DYN.tire_radius*60/(2 * math.pi))
        self.GGV = GGV.GGV(self.AERO, self.DYN, self.PTN, self.gear_tot, self.v_max)
        
        self.trajectory = Trajectory.Trajectory(trajectory_path, self.GGV.radii_range[0], self.GGV.radii_range[-1])
        self._interval = 10

        self._mesh_size = self.trajectory.num_points * self._interval
        self.count = np.zeros(self._mesh_size)
        self.dist = np.zeros(self._mesh_size)
        self.velocity = np.zeros(self._mesh_size)
        self.gear = np.zeros(self._mesh_size)
    
    def simulate_forwards(self, starting_v):
        vel = starting_v
        gear = 1
        required_gear = gear
        time_shifting = 0
        is_shifting = False
        count = 0
        distance = 0

        for point_idx in range(self.trajectory.num_points - 1):
            x_1 = self.trajectory.points[0][point_idx]
            y_1 = self.trajectory.points[1][point_idx]
            x_2 = self.trajectory.points[0][point_idx + 1]
            y_2 = self.trajectory.points[1][point_idx + 1]

            '''Distance of trajectory interval in meters'''
            dist = math.sqrt((x_1-x_2)**2 + (y_2-y_1)**2)

            r = self.trajectory.radii[point_idx]

            '''Max Achievable Cornering Velocity'''
            v_max = min(self.v_max, self.GGV.cornering_capability.evaluate(r))

            for i in range(len(self.GGV.expected_gears)):
                if(self.GGV.expected_gears[i] > vel):
                    required_gear = self.GGV.expected_gears[i]
            
            is_shifting = required_gear > gear

            AX_cap = self.GGV.accel_capability.evaluate(vel)
            AY_cap = self.GGV.lateral_capability.evaluate(vel)
            AY_actual = vel ** 2 / (r * 9.81)

            delta_d = dist / self._interval

            for j in range(self._interval):
                dt = delta_d / vel
                if is_shifting and vel < v_max:
                    # Currently shifting, do not accelerate
                    dt = delta_d / vel
                    time_shifting += dt
                elif vel < v_max:
                    ax_potential = AX_cap*(1-(min(AY_cap,AY_actual)/AY_cap)**2)
                
                    p = Polynomial([-delta_d, vel, 0.5*9.81*ax_potential])
                    tt = p.roots()
                    dt = max(tt)

                    dv = 9.81 * ax_potential * dt
                    vel = min(v_max, vel + dv)

                    for i in range(len(self.GGV.expected_gears)):
                        if(self.GGV.expected_gears[i] > vel):
                            required_gear = self.GGV.expected_gears[i]
                    
                    is_shifting = required_gear > gear
                else:
                    # Vehicle is neither shifting or accelerating, is at vmax
                    vel = v_max
                    dt = delta_d / vel

                

                if time_shifting >= self.PTN.shift_time:
                    is_shifting = False
                    time_shifting = 0
                    gear = required_gear

                self.count[count] = count
                self.dist[count] = distance
                self.velocity[count] = vel
                self.gear[count] = gear

                count += 1
                distance += delta_d

        fig, ax = plt.subplots()
        ax.plot(self.dist, self.velocity)
        plt.show()
        c = []
        for i in range(self._mesh_size):
            if i % self._interval == 0:
                c.append(self.gear[i])
            
        plt.scatter(self.trajectory.points[0], self.trajectory.points[1], c=c)
        plt.colorbar()
        plt.show()

if __name__ == "__main__":
    v = Vehicle()
    #print(len(v.PTN.rpm_range))
    #print(len(v.PTN.torque_curve))
    v.GGV.generate()
    t = Trajectory.Trajectory("./trajectory/17_lincoln_endurance_track_highres.xls", v.GGV.radii_range[0], v.GGV.radii_range[-1])
    plt.scatter(t._trajectory[0], t._trajectory[1], c=t._radii)
    plt.colorbar()
    plt.show()

