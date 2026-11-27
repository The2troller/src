#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3" ^
import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry

class Explorer_node(Node): 

    def __init__(self):
        super().__init__("explore_node")
        self.setup_parameters()
        self.init_communications()
        
    def setup_parameters(self):
        self.max_angle = 28.5
        self.rotate_msg = Twist()
        self.rotate_msg.angular.z = 0.8
        self.foward_msg = Twist()
        self.foward_msg.linear.x = 0.2
        self.F_RANGE = 20.0
        self.x = None
        self.y = None
        self.front_rs = []
        self.fowarding = False
        self.rotating = False
        self.theta = None
        self.last_x = None
        self.last_y = None
        self.last_theta = None
        self.STEP = 0.2
        self.turn_dir = 90.0
        self.last_was_rotate = False

    def init_communications(self):
        self.publisher_displacement = self.create_publisher(Pose, "/displacement", 10 )
        self.subscription_client = self.create_subscription(Empty, "/explore", self.decide_move, 1)
        self.publisher_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription_odom = self.create_subscription(Odometry, "/odom", self.odom_rec, 1)
        self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.scan_rec, 1)
        self.foward_timer = self.create_timer(0.05, self.foward_handler)
        self.rotate_timer = self.create_timer(0.05, self.rotate_handler)
        
    def decide_move(self, msg):
        if self.fowarding or self.rotating:
            return
        if self.x == None or self.y == None or self.theta == None:
            return
            
        self.last_x = self.x
        self.last_y = self.y
        self.last_theta = self.theta

        if self.front_rs == []:
            self.rotating = False
            self.fowarding = True
            self.last_was_rotate = False
        else:
            if min(self.front_rs) < 0.5:
                if not self.last_was_rotate:
                    self.turn_dir = float(np.random.choice([-90.0, 90.0]))
                self.target_angle = self.theta + self.turn_dir
                self.last_was_rotate = True
                self.rotating = True
                self.fowarding = False
            else:
                self.rotating = False
                self.fowarding = True
                self.last_was_rotate = False

    def foward_handler(self):
        if self.fowarding == True:
            if np.hypot(self.x - self.last_x, self.y - self.last_y) >= self.STEP:
                self.fowarding = False
                d_moved = self.get_displacement()
                self.publisher_displacement.publish(d_moved)
                self.publisher_vel.publish(Twist())
                return
            if self.front_rs == []:
                self.publisher_vel.publish(self.foward_msg)
            elif min(self.front_rs) >= 0.5:
                self.publisher_vel.publish(self.foward_msg)
            else:
                self.fowarding = False
                d_moved = self.get_displacement()
                self.publisher_displacement.publish(d_moved)
                self.publisher_vel.publish(Twist())

    def rotate_handler(self):
        if self.rotating == True:
            err = (self.target_angle - self.theta + 180.0) % 360.0 - 180.0
            if abs(err) > 2.0:
                self.rotate_msg.angular.z = float(np.copysign(0.8 if abs(err) > 15.0 else 0.3, err))
                self.publisher_vel.publish(self.rotate_msg)
            else:
                self.rotating = False
                d_moved = self.get_displacement()
                self.publisher_displacement.publish(d_moved)
                self.publisher_vel.publish(Twist())

    def get_displacement(self):  
        dx = self.x - self.last_x
        dy = self.y - self.last_y
        dtheta = (self.theta - self.last_theta + 180.0) % 360.0 - 180.0
        rad_init = np.deg2rad(self.last_theta)
        d_moved = Pose()
        d_moved.position.x = float(dx * np.cos(rad_init) + dy * np.sin(rad_init))
        d_moved.position.y = float(-dx * np.sin(rad_init) + dy * np.cos(rad_init))
        d_moved.orientation.z = float(np.sin(np.deg2rad(dtheta) / 2.0))
        d_moved.orientation.w = float(np.cos(np.deg2rad(dtheta) / 2.0))
        return d_moved

    def odom_rec(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        sin_ = 2.0 * (q.w * q.z + q.x * q.y)
        cos_ = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = np.rad2deg(np.arctan2(sin_, cos_))
        
    def scan_rec(self, msg: LaserScan):
        self.angle_inc = msg.angle_increment
        self.ranges = msg.ranges
        self.max_d = msg.range_max
        self.angle_min = msg.angle_min
        self.valid_rays = list()
        for i in range(len(self.ranges)):
            theta = np.rad2deg(self.angle_min + i * self.angle_inc)
            r = self.ranges[i]
            if abs(theta) <= self.max_angle and r < self.max_d:
                self.valid_rays.append([theta, r])
        self.front_rs = list()
        for theta, r in self.valid_rays:
            if (theta > -self.F_RANGE) and (theta < self.F_RANGE):
                self.front_rs.append(r)
def main(args=None):
    rclpy.init(args=args) #? Inicializa ROS
    node = Explorer_node() #? Instancia de un nodo de clase MyNode
    rclpy.spin(node) #? Permite ejecución continua de un nodo
    rclpy.shutdown() #? Cierra el nodo al terminar la ejecución
    
if __name__ == "__main__":
    main()