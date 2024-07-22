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

                # Apply potential field for obstacle avoidance
                f_attr_x, f_attr_y = self.attractive_force(x_target, y_target)
                f_rep_x, f_rep_y = self.repulsive_force()
                f_x = f_attr_x + f_rep_x
                f_y = f_attr_y + f_rep_y
                
                print("f_x: ", f_x, " f_y: ", f_y)

                v_target, w_target = self.compute_velocity(f_x, f_y)

                vl = v_target - w_target * self.wheel_base / 2
                vr = v_target + w_target * self.wheel_base / 2

                # convert speeds from m/s to deg/sec
                self.vl = vl / self.wheel_radius
                self.vr = vr / self.wheel_radius

            self.t += self.delta_t

        finally:
            self.mutex.release()

    def attractive_force(self, x_target, y_target):
        K_att = 0.35  # Gain for attractive force
        f_attr_x = K_att * (x_target - self.x)
        f_attr_y = K_att * (y_target - self.y)
        return f_attr_x, f_attr_y

    def repulsive_force(self):
        K_rep = .8  # Gain for repulsive force
        repulsive_threshold = 0.2  # Distance threshold for repulsive force
        epsilon = 1e-6  # Piccolo valore per evitare divisioni per zero

        f_rep_x, f_rep_y = 0.0, 0.0
        if self.obstacles:
            tmp = self.obstacles.process()
            
            tuple_list = [tmp[i:i+3] for i in range(0, len(tmp), 3)]
            for obs in tuple_list:
                obs_x, obs_y, obs_r = obs  # (x, y, radius)
                dx = self.x - obs_x
                dy = self.y - obs_y
                dist_to_center = math.sqrt(dx * dx + dy * dy)
                dist_to_edge = dist_to_center - obs_r  # Distance to the edge of the obstacle

                if dist_to_edge < repulsive_threshold and dist_to_edge > epsilon:
                    rep_force = K_rep * (1.0 / dist_to_edge - 1.0 / repulsive_threshold) * (1.0 / (dist_to_edge * dist_to_edge))
                    f_rep_x += rep_force * (dx / dist_to_center)
                    f_rep_y += rep_force * (dy / dist_to_center)
        
        return f_rep_x, f_rep_y


    def compute_velocity(self, f_x, f_y):
        v_target = math.sqrt(f_x * f_x + f_y * f_y)
        w_target = math.atan2(f_y, f_x) - self.theta
        return v_target, w_target

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
