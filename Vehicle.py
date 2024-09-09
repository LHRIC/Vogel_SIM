from matplotlib import pyplot as plt
import ggv.GGV as GGV
import setups
import state_models
import trajectory.Trajectory as Trajectory
from numpy.polynomial import Polynomial
import numpy as np
import math


class Vehicle:
    def __init__(self, trajectory_path, params, is_closed, mesh_resolution=10):
        self.params = params

        '''Total Reduction'''
        self.gear_tot = self.params.gear_ratios[-1] * self.params.final_drive * self.params.primary_reduction
        '''Max Achievable Velocity, assuming RPM never exceeds shiftpoint'''
        self.v_max = self.params.shiftpoint / (self.gear_tot/self.params.tire_radius*60/(2 * math.pi))
        self.GGV = GGV.GGV(self.params, self.gear_tot, self.v_max)
        
        

        self.trajectory = Trajectory.Trajectory(trajectory_path, is_closed, self.GGV.radii_range[0], self.GGV.radii_range[-1])
        self._interval = mesh_resolution

        self._mesh_size = (self.trajectory.num_points - 1) * self._interval
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
        self.is_shifting = np.zeros(self._mesh_size)

        self.ax = np.zeros(self._mesh_size)
        self.ax_f = np.zeros(self._mesh_size)
        self.ax_r = np.zeros(self._mesh_size)

        self.ay = np.zeros(self._mesh_size)
        self.ay_f = np.zeros(self._mesh_size)
        self.ay_r = np.zeros(self._mesh_size)
        self.fz_fr = []
        self.fz_fl = []
        self.fz_rr = []
        self.fz_rl = []
        self.fz_total = []
        self.cgz = []

    
        def __del__(self):
            self._matlab_engine.quit()


    def reset_ggv(self):
        self.GGV = GGV.GGV(self.AERO, self.DYN, self.PTN, self.params, self.gear_tot, self.v_max, self._matlab_engine)

    def simulate_accel(self):
        vel = 1e-6
        gear = 1
        required_gear = gear
        time_shifting = 0
        is_shifting = False
        count = 0
        distance= 0
        time = 0

        for d in range(75):
            #x_1 = self.trajectory.points[0][point_idx]
            #x_2 = self.trajectory.points[0][(point_idx + 1) % self.trajectory.num_points]

            '''Distance of trajectory interval in meters'''
            dist = 1

            for idx, v in enumerate(self.GGV.velocity_range):
                if(v > vel):
                    required_gear = self.GGV.expected_gears[idx]
                    break
            
            is_shifting = required_gear != gear

            AX_cap = self.GGV.accel_capability.evaluate(vel)
            delta_d = dist / self._interval

            for j in range(self._interval):
                self.x[count] = d + dist * j/self._interval
                self.y[count] = 0 * j/self._interval

                self.ay[count] = 0

                self.turn_dir[count] = 1

                self.count[count] = count
                self.velocity[count] = vel
                self.gear[count] = gear
                self.dist[count] =  distance + delta_d * j

                v_max = self.v_max - 1

                dt = delta_d / vel
                self.time[count] = time + dt * j
                if is_shifting and vel < v_max:
                    # Currently shifting, do not accelerate
                    dt = delta_d / vel
                    time_shifting += dt
                    self.ax[count] = 0
                    self.is_shifting[count] = 1
                elif vel < v_max:
                    # Take a slice of the GGV using AX_cap and AY_cap as verticies
                    # the actual lateral acceleration will define the remaining longitudinal acceleration.
                    self.ax[count] = AX_cap

                    # Solve the 1D kinematic equation for time
                    p = Polynomial([-delta_d, vel, 0.5*9.81*AX_cap])
                    tt = p.roots()
                    dt = max(tt)

                    #Solve for change in velocity
                    dv = 9.81 * AX_cap * dt

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
                    self.ax[count] = 0

                

                if time_shifting >= self.params.shift_time:
                    is_shifting = False
                    time_shifting = 0
                    gear = required_gear
                
                count += 1
                time += dt
            
            distance += dist

        return max(self.time)

    def simulate_endurance(self):
        vel = 20 * 0.3048; # m/s
        self.simulate_forwards(vel)
        self.simulate_reverse()
        self.simulate_forwards(self.velocity_r[0])

        start_vel = []
        end_vel = []
        in_brake = False

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
            
            '''Snippet for Little Liam I think, some brakes stuff idr'''
            if(i > 0):
                if self.ax[i] < 0 and self.ax[i-1] > 0:
                    start_vel.append(self.velocity[i-1])
                    in_brake = True
                elif (in_brake) and (self.ax[i] > 0 and self.ax[i-1] <= 0):
                    end_vel.append(self.velocity[i-1])
                    in_brake = False
        

        '''Determine turn handedness and remove outliers'''
        self.ay[self.ay > self.GGV.lateral_capability.evaluate(self.v_max + 1)] = self.GGV.lateral_capability.evaluate(self.v_max + 1)
        self.ay_f[self.ay_f > self.GGV.lateral_capability.evaluate(self.v_max + 1)] = self.GGV.lateral_capability.evaluate(self.v_max + 1)
        self.ay_r[self.ay_r > self.GGV.lateral_capability.evaluate(self.v_max + 1)] = self.GGV.lateral_capability.evaluate(self.v_max + 1)
        
        self.ay = np.multiply(self.ay, self.turn_dir)
        self.ay_f = np.multiply(self.ay_f, self.turn_dir)
        self.ay_r = np.multiply(self.ay_r, self.turn_dir)
        
        self.ax_r = np.multiply(self.ax_r, -1)
        
        
        # self.ax.plot(self.dist_f, math.comb)
        # plt.show()
        # plt.scatter(self.x, self.y, c=math.comb)
        # plt.colorbar()
        # plt.show()
        # fig, ax = plt.subplots()
        # ax.scatter(self.x,self.y,marker=',')
        # for i, txt in enumerate(range(len(self.x))):
        #     ax.annotate(txt,(self.x[i],self.y[i]))

        print('POINTS',len(self.velocity))

        # Gathering data for plots
        self.cgz=[]
        self.vtest=[]
        self.roll=[]
        self.pitch=[]
        self.axp=[]
        self.ayp=[]
        self.xpos = []
        self.ypos = []

        for i in range(self.trajectory.num_points - 1) :
            i = int(i)
            r = self.trajectory.radii[i]
            state_in = state_models.StateInput(Ax=self.ax[i], Ay=self.ay[i], v=self.velocity[i], r=r, delta=0, beta=0)
            # state_in = state_models.StateInput(Ax=0, Ay=0, v=self.velocity[i], r=r, delta=0, beta=0) # Constant accel variation
            self.setup = setups.Panda
            v =state_models.VehicleState(params=self.params)
            v.eval(state_in=state_in)
            # self.fz_rr.append(v.rr_tire.Fz)
            # self.fz_rl.append(v.rl_tire.Fz)
            # self.fz_fr.append(v.fr_tire.Fz)
            # self.fz_fl.append(v.fl_tire.Fz)
            # self.fz_total.append(v.rr_tire.Fz)
            # self.fz_total.append(v.rl_tire.Fz)
            # self.fz_total.append(v.fr_tire.Fz)
            # self.fz_total.append(v.fl_tire.Fz)
            self.cgz.append(v.cgz)
            self.vtest.append(v.v)
            self.roll.append(v.phi)
            self.pitch.append(v.theta)
            self.axp.append(v.Ax)
            self.ayp.append(v.Ay)
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
        for point_idx in (range(self.trajectory.num_points - 1)):
            x_1 = self.trajectory.points[0][point_idx]
            y_1 = self.trajectory.points[1][point_idx]
            x_2 = self.trajectory.points[0][(point_idx + 1)]
            y_2 = self.trajectory.points[1][(point_idx + 1)]
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

                self.ay_f[count] = min(AY_cap,AY_actual)

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
                    self.is_shifting[count] = 1
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

                

                if time_shifting >= self.params.shift_time:
                    is_shifting = False
                    time_shifting = 0
                    gear = required_gear
                
                count += 1
                time += dt

            distance += dist

    def simulate_reverse(self):
        vel = self.velocity_f[-1]
        distance = 0
        count = int(self.count[-1])
        for point_idx in range(self.trajectory.num_points - 1, -1, -1):
            x_1 = self.trajectory.points[0][point_idx]
            y_1 = self.trajectory.points[1][point_idx]
            x_2 = self.trajectory.points[0][(point_idx - 1)]
            y_2 = self.trajectory.points[1][(point_idx - 1)]

            '''Distance of trajectory interval in meters'''
            dist = math.sqrt((x_1-x_2)**2 + (y_2-y_1)**2)

            r = self.trajectory.radii[point_idx]

            '''Max Achievable Cornering Velocity through this segment'''
            v_max = min(self.v_max, self.GGV.cornering_capability.evaluate(r))

            AX_cap = -1 * self.GGV.braking_capability.evaluate(vel)
            AY_cap = self.GGV.lateral_capability.evaluate(vel)
            AY_actual = vel ** 2 / (r * 9.81)

            delta_d = dist/self._interval
            for j in range(self._interval):
                self.velocity_r[count] = vel
                self.dist_r[count] =  distance + delta_d * j

                self.ay_r[count] = min(AY_cap,AY_actual)
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

