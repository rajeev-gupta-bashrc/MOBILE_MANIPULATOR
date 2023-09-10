import math
import matplotlib.pyplot as plt
import time 
import numpy as np


class Path_Planner():
    def __init__(self, x_min, y_min, x_max, y_max, function=''):
        self.function = function
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.max_iter = 1000
        self.step_size = 0.1
        self.x_init = self.x_min
        self.x_closest = 1e10
        self.y_closest = 1e10
        self.o_closest = 1e10           #theta
        self.x_goal = 1e10
        self.y_goal = 1e10
        self.dist_tol = 0.01
        self.t_step = 0.010
        self.drive = None
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X-axis')
        self.ax.set_ylabel('Y-axis')
        self.ax.set_title('Robot Position and Trajectory in 2-Dimension')
        self.one_time_legend = True
    
    def set_x_init(self, x_init):
        self.x_init = x_init
        
    def f(self, x):
        return eval(compile(self.function, "<string>", "eval"))
    
    def get_distance(self, x1, y1, x2, y2):
        try:
            return ((x1-x2)**2+(y1-y2)**2)**0.5
        except OverflowError:
            return 1e10
        
    def get_gradient(self, x, y, x0, y0):
        try:
            return ((x-x0) + 2*x*(self.f(x)-y0))/(((x-x0)**2+(self.f(x)-y0)**2)**0.5)
        except (ZeroDivisionError, OverflowError):
            return 0
        
    def get_closest_point(self, x0, y0):
        x_final = x0 if self.x_closest == 1e10 else self.x_closest
        dist = self.dist_tol
        t = 0
        cond = True
        osc = 0
        last_track = [0 for i in range(3)]
        while(cond and t < self.max_iter):
            t+=1
            x_final -= self.step_size * self.get_gradient(x_final, self.f(x_final), x0, y0)
            last_track[0] = last_track[1]
            last_track[1] = last_track[2]
            last_track[2] = x_final
            if t>100:
                if last_track[0]>0 and last_track[2]>0 and last_track[1]<0 or \
                   last_track[0]<0 and last_track[2]<0 and last_track[1]>0:
                    osc += 1
            cond = osc<10
        self.x_closest = x_final
        self.y_closest = self.f(x_final)
        # print(x_final, self.f(x_final), t)
        return x_final, self.f(x_final)
    
    def get_normal_vector(self):
        x0, y0 = self.drive.posn[0], self.drive.posn[1]
        x1, y1 = self.x_closest, self.y_closest
        # print([(x1-x0)/mod_vector, (y1-y0)/mod_vector])
        try:
            mod_vector = ((x1-x0)**2 + (y1-y0)**2)**0.5
            return [(x1-x0)/mod_vector, (y1-y0)/mod_vector]
        except (ZeroDivisionError, OverflowError):
            return [0.0, 0.0]
    
    def get_tanget_vector(self):
        x0, y0 = self.drive.posn[0], self.drive.posn[1]
        x2 = x0 + self.dist_tol if self.x_goal-x0 > 0 \
                                            else x0 - self.dist_tol
        y2 = self.f(x2)
        try:
            mod_vector = ((x2-x0)**2 + (y2-y0)**2)**0.5
            return [(x2-x0)/mod_vector, (y2-y0)/mod_vector]
        except (ZeroDivisionError, OverflowError):
            return [0.0, 0.0]
        
        
    def get_cmd(self):
        odom = self.drive.posn
        _x_closest, _y_closest = self.get_closest_point(odom[0], odom[1])
        _dist = self.get_distance(odom[0], odom[1], _x_closest, _y_closest)
        normal_vector = self.get_normal_vector()
        tangent_vector = self.get_tanget_vector()
        mf = 1/(1+(math.e)**(-30*(_dist-0.20)))
        normal_vector[0], normal_vector[1] = normal_vector[0]*mf, normal_vector[1]*mf
        tangent_vector[0], tangent_vector[1] = tangent_vector[0]*(1-mf), tangent_vector[1]*(1-mf)
        return [normal_vector[0] + tangent_vector[0], normal_vector[1] + tangent_vector[1]]
    
    def move_to(self, x_goal, y_goal):
        # position of goal  
        self.ax.plot(x_goal, y_goal, 'go', label = 'Goal Position', ms=4)
        
        self.x_goal = x_goal
        self.y_goal = y_goal
        x0, y0 = self.drive.posn[0], self.drive.posn[1]
        dist = self.get_distance(x0, y0, self.x_goal, self.y_goal)
        
        # to track the line joining the goal and the bot
        if dist > self.step_size:
            m = (y_goal-y0)/(x_goal-x0) if x_goal!=x0 else 1e100
            c = y0 - m * x0
            self.function = f"{m}*x + {c}"
            self.follow()
        dist = self.get_distance(self.drive.posn[0], self.drive.posn[1], self.x_goal, self.y_goal)
        
        # to reach the goal once the bot is sufficiently near to the goal
        while(dist > self.dist_tol):
            print(self.drive.posn)
            vector = [0.0, 0.0]
            try:
                vector = [-(self.drive.posn[0]-self.x_goal)/dist, -(self.drive.posn[1]-self.y_goal)/dist]
            except (ZeroDivisionError, OverflowError) as e:
                print(e)
                plt.show()
            self.drive.move(vector)
            time.sleep(self.t_step)
            dist = self.get_distance(self.drive.posn[0], self.drive.posn[1], self.x_goal, self.y_goal)
            
            self.ax.plot(self.drive.posn[0], self.drive.posn[1], 'bo', label = 'Robot\'s Position', ms=1)
            self.drive.forward_vel = self.drive.max_linear_vel  * ( 2/(1+math.e**(-dist)) - 1)
            self.drive.angular_vel = self.drive.max_angular_vel * ( 2/(1+math.e**(-dist)) - 1)
            
        print("Destination arrived: Please ensure the safety of your belongings by your own.")
        plt.legend()        
        plt.show()
        
    def follow(self):
        # plt.figure()
        show_here = False
        delta = self.step_size
        x_desired = 0.0
        y_desired = self.f(x_desired)
        
        # plt.xlim(self.x_min, self.x_max)
        # plt.ylim(self.y_min, self.y_max)
        dist = self.get_distance(self.drive.posn[0], self.drive.posn[1], self.x_goal, self.y_goal)
        try:
            while(dist>self.step_size):
                print(self.drive.posn)
                self.drive.move(self.get_cmd())
                
                # sleep and plot
                plt.pause(self.t_step)
                self.ax.plot(self.drive.posn[0], self.drive.posn[1], 'bo', label = 'Robot\'s Position', ms=1)
                if self.drive.posn[0] + 1 > x_desired:
                    self.ax.plot(x_desired, y_desired, 'ro-', label = 'Desired Path', ms=1)
                    
                    # to call legend() one time only
                    if self.one_time_legend is True:
                        self.ax.legend()
                        self.one_time_legend = False
                        
                    x_desired += delta 
                    y_desired = self.f(x_desired)
                
                dist = self.get_distance(self.drive.posn[0], self.drive.posn[1],
                                         self.x_goal, self.y_goal)
                self.drive.forward_vel = self.drive.max_linear_vel  * ( 2/(1+math.e**(-dist)) - 1) 
                self.drive.angular_vel = self.drive.max_angular_vel * ( 2/(1+math.e**(-dist)) - 1)
                
        except KeyboardInterrupt:
            show_here = True
            print("Showing the result till now.")
            
        if show_here: 
            plt.legend()
            plt.show()
            
        
    def set_vehicle(self, vehicle):
        self.drive = vehicle
    