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
        self.rotate_msg.angular.z = 0.1
        self.foward_msg = Twist()
        self.foward_msg.linear.x = 0.1
        self.F_RANGE = 10.0
        self.x = None
        self.y = None
        self.front_rs = []

    def init_communications(self):
        self.publisher_displacement = self.create_publisher(Pose, "/displacement", 10 )
        self.subscription_client = self.create_subscription(Empty, "/explore", self.decide_move, 1)
        self.publisher_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription_odom = self.create_subscription(Odometry, "/odom", self.odom_rec, 1)
        self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.scan_rec, 1)
        self.foward_timer = self.create_timer(1.0, self.foward_handler)
        self.rotate_timer = self.create_timer(1.0, self.rotate_handler)
        
    def decide_move(self, msg):
        if self.x == None or self.y == None or self.front_rs == []:
            return
            
        last_x = self.x
        last_y = self.y
        
        if min(self.front_rs) < 0.5:
            self.target_angle = self.theta + 90.0
            self.rotating = True
            self.fowarding = False
        else:
            self.rotating = False
            self.fowarding = True

    def foward_handler(self):
        if self.fowarding == True:
            if min(self.front_rs) >= 0.5:
                self.publisher_vel.publish(self.foward_msg)
            else:
                self.fowarding = False
                #add send displacement
                #d_moved = Pose()
                #self.publisher_displacement.publish(d_moved)
    def rotate_handler(self):
        if self.rotating == True
            err = self.target_angle - self.theta
            if abs(err) > 1.0:
                self.publisher_vel.publish(self.rotate_msg)
            else:
                self.rotating = False
                #add send displacement
                #d_moved = Pose()
                #self.publisher_displacement.publish(d_moved)

    def odom_rec(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        sin_ = 2.0 * (q.w * q.z + q.x * q.z)
        cos_ = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = np.atan2(sin_, cos_)
        
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
