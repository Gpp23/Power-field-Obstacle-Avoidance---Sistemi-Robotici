import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}")

from lib.godot.interface import *
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, StraightLine2DMotion
from lib.data.plot import DataPlotter

import os
import threading
import time


class GodotObstacles(GodotInterface):
    
    def __init__(self, uPort = 4445):
        super().__init__(uPort)

    def process(self):
        packet = struct.pack("<f", 1)
        self.sd.sendto(packet, ('localhost', self.port))
        (reply, remote) = self.sd.recvfrom(1024)
        (xa,ya,ra,xb,yb,rb,xc,yc,rc) = struct.unpack("<fffffffff", reply[8:])
        return (xa,ya,ra,xb,yb,rb,xc,yc,rc)
    
class GodotTarget(GodotInterface):
    
    def __init__(self, uPort = 4446):
        super().__init__(uPort)

    def process(self):
        packet = struct.pack("<f", 1)
        self.sd.sendto(packet, ('localhost', self.port))
        (reply, remote) = self.sd.recvfrom(1024)
        (x,y) = struct.unpack("<ff", reply[8:])
        return (x,y)
    
    
class PowerFieldObstacleAvoidance():
    
    def __init__(self , _K_att = 0.35, _K_rep = .8, _repulsive_threshold = 0.2, _fmax = 1.5):
        self.K_att = _K_att
        self.K_rep = _K_rep
        self.repulsive_threshold = _repulsive_threshold
        self.fmax = _fmax

    def evaluate(self, theta, x_target, y_target, x, y, obstacles):
        # Apply potential field for obstacle avoidance
        f_attr_x, f_attr_y = self.attractive_force(x_target, y_target, x, y)
        f_rep_x, f_rep_y = self.repulsive_force(obstacles, x, y)
        f_x = f_attr_x + f_rep_x
        f_y = f_attr_y + f_rep_y
        
        if (f_x > self.fmax):
            f_x = self.fmax
        elif (f_x < -self.fmax):
            f_x = -self.fmax
            
        if (f_y > self.fmax):
            f_y = self.fmax
        elif (f_y < -self.fmax):
            f_y = -self.fmax
            
        print("f_x: ", f_x, " f_y: ", f_y)

        v_target, w_target = self.compute_velocity(f_x, f_y, theta)
        
        return v_target, w_target
    
    def attractive_force(self, x_target, y_target, x, y):
        f_attr_x = self.K_att * (x_target - x)
        f_attr_y = self.K_att * (y_target - y)
        
        print("f_attr_x: ", f_attr_x, "f_attr_y: ", f_attr_y)
        return f_attr_x, f_attr_y

    def repulsive_force(self, obstacles, x, y):
        epsilon = 1e-6 
        
        f_rep_x, f_rep_y = 0.0, 0.0
        if obstacles:
            tmp = obstacles.process()
            
            tuple_list = [tmp[i:i+3] for i in range(0, len(tmp), 3)]
            for obs in tuple_list:
                obs_x, obs_y, obs_r = obs  # (x, y, radius)
                dx = x - obs_x
                dy = y - obs_y
                dist_to_center = math.sqrt(dx * dx + dy * dy)
                dist_to_edge = dist_to_center - obs_r  # Distance to the edge of the obstacle

                if dist_to_edge < self.repulsive_threshold and dist_to_edge > epsilon:
                    rep_force = self.K_rep * (1.0 / dist_to_edge - 1.0 / self.repulsive_threshold) * (1.0 / (dist_to_edge * dist_to_edge))
                    f_rep_x += rep_force * (dx / dist_to_center)
                    f_rep_y += rep_force * (dy / dist_to_center)
                    
                    print("f_rep_x: ", f_rep_x, "f_rep_y: ", f_rep_y)
        
        return f_rep_x, f_rep_y
    
    def compute_velocity(self, f_x, f_y, theta):
        v_target = math.sqrt(f_x * f_x + f_y * f_y)
        w_target = math.atan2(f_y, f_x) - theta
        return v_target, w_target

class Cart2DRobot(threading.Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None):
        super(Cart2DRobot, self).__init__(group=group, target=target,
                                          name=name)

        self.daemon = True
        self.mutex = threading.Lock()

        self.t = 0
        self.cart = GodotCartTwoWheels()

        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.w = 0

        self.vl = 0
        self.vr = 0

        self.wheel_base = 0.32
        self.wheel_radius = 0.051

        self.active = True

        self.targets = None  # Object interfacing with Godot to get target positions
        self.obstacles = None  # Object interfacing with Godot to get obstacle positions and radius
        
        self.pfoa = PowerFieldObstacleAvoidance(.2, .05, .5, 10) # K_att, K_rep, Repulsive_threshold

    def goto(self, x, y):
        try:
            self.mutex.acquire()
            self.trajectory.start_motion((self.x, self.y), (x, y))
            self.active = True
        finally:
            self.mutex.release()

    def stop(self):
        self.vl = 0
        self.vr = 0
        self.active = False

    def run(self):
        # important task, set maximum priority
        # param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
        # os.sched_setscheduler(0, os.SCHED_FIFO, param)
        while True:
            time.sleep(0.001)
            self.__execute()

    def __execute(self):
        try:
            self.mutex.acquire()

            (self.delta_t, self.x, self.y, self.theta, self.v, self.w) = self.cart.process(self.vl, self.vr)

            if self.active:

                (x_target, y_target) = self.targets.process()
                
                dist = math.sqrt((self.x - x_target)**2 + (self.y - y_target)**2)
                
                if (dist < 0.1):
                    print("Target raggiunto!")
                    self.stop()
                    return
                
                v_target, w_target = self.pfoa.evaluate(self.theta, x_target, y_target, self.x, self.y, self.obstacles)

                vl = v_target - w_target * self.wheel_base / 2
                vr = v_target + w_target * self.wheel_base / 2

                # convert speeds from m/s to deg/sec
                self.vl = vl / self.wheel_radius
                self.vr = vr / self.wheel_radius

            self.t += self.delta_t

        finally:
            self.mutex.release()

    def get_pose_deg(self):
        try:
            self.mutex.acquire()
            return (self.x, self.y, math.degrees(self.theta))
        finally:
            self.mutex.release()

    def get_pose(self):
        try:
            self.mutex.acquire()
            return (self.x, self.y, self.theta)
        finally:
            self.mutex.release()

    def set_targets(self, targets):
        try:
            self.mutex.acquire()
            self.targets = targets
        finally:
            self.mutex.release()

    def set_obstacles(self, obstacles):
        try:
            self.mutex.acquire()
            self.obstacles = obstacles
        finally:
            self.mutex.release()


if __name__ == '__main__':
    cart_robot = Cart2DRobot()

    target = GodotTarget() 
    obstacles = GodotObstacles()


    cart_robot.set_targets(target)
    cart_robot.set_obstacles(obstacles)

    while True:
        time.sleep(1)
        cart_robot.run()
