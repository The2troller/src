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
        self.init_communications()
        self.setup_parameters()

    def setup_parameters(self):
        self.max_angle = 28.5
        self.rotate_msg = Twist()
        self.rotate_msg.angular.z = 1.0
        self.foward_msg = Twist()
        self.foward_msg.linear.x = 1.0

    def init_communications(self):
        self.publisher_displacement = self.create_publisher(Pose, "/displacement", 10 ) #cambiar tipo de msg
        self.subscription_client = self.create_subscription(Empty, "/explore", self.decide_move, 1)
        self.publisher_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription_odom = self.create_subscription(Odometry, "/odom", self.odom_rec, 1)
        self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.scan_rec, 1)
        
    def decide_move(self, msg):
        valid_rays = list()
        for i in range(len(self.ranges)):
            theta = np.rad2deg(self.angle_min + i * self.angle_inc)
            r = self.ranges[i]
            if abs(theta) <= self.max_angle and r < self.max_d:
                valid_rays.append([theta, r])
        

        f_wall = False
        for theta, r in valid_rays:
            if (theta > -10.0) and (theta < 10.0) and r < 1.0:
                wall = True


        if wall == True:
            self.publisher_vel.publish(self.rotate_msg)
            self.publisher_displacement.publish(self.rotate_msg)
        else:
            self.publisher_vel.publish(self.foward_msg)
            self.publisher_displacement.publish(self.foward_msg)


    def odom_rec(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def scan_rec(self, msg: LaserScan):
        self.angle_inc = msg.angle_increment
        self.ranges = msg.ranges
        self.max_d = msg.range_max
        self.angle_min = msg.angle_min


def main(args=None):
    rclpy.init(args=args) #? Inicializa ROS
    node = Explorer_node() #? Instancia de un nodo de clase MyNode
    rclpy.spin(node) #? Permite ejecución continua de un nodo
    rclpy.shutdown() #? Cierra el nodo al terminar la ejecución
    
if __name__ == "__main__":
    main()