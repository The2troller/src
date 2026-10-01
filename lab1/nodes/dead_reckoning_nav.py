#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3" ^
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
import time
import numpy as np
from threading import Thread

class Movement_Node(Node):

    def __init__(self):
        super().__init__("node_name")
        self.init_communications()
        self.setup_parameters()
        
    def setup_parameters(self):
        self.velocity = Twist()
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.vel = 0.2
        self.turn_vel = 1.0
        self.pos_odo_active = False
        self.pos_real_active = False
        self.say_pos_timer = self.create_timer( 2.0, self.say_pos )
        self.offset = 0.0
        self.predict_pose = [0, 0, 0]

    def init_communications(self):
        self.publisher = self.create_publisher( Twist, "/cmd_vel", 10 )
        self.subscription_goal = self.create_subscription(PoseArray, "goal_list", self.start_moving_node, 1)
        self.subscription_odometry = self.create_subscription(Odometry, "/odom", self.read_odometry, 1)
        self.subscription_real = self.create_subscription(Pose, "/real_pose", self.read_real, 1)

    def aplicar_velocidad(self, speed_command_list):
        func_rate = self.create_rate(100)
        for command in speed_command_list:
            init_time = self.get_clock().now().nanoseconds / 1e9
            self.velocity.linear.x = command[0]
            self.velocity.angular.z = command[1]
            current_time = self.get_clock().now().nanoseconds / 1e9
            last_cycle_time = current_time
            while self.get_clock().now().nanoseconds / 1e9 - init_time <= command[2]:
                current_time = self.get_clock().now().nanoseconds / 1e9
                time_passed = current_time - last_cycle_time
                last_cycle_time = current_time
                self.predict_pose[0] += time_passed * command[0] * np.cos(self.predict_pose[2])
                self.predict_pose[1] += time_passed * command[0] * np.sin(self.predict_pose[2])
                self.predict_pose[2] += time_passed * command[1]
                self.publisher.publish(self.velocity)
                func_rate.sleep()
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.publisher.publish(self.velocity)
        
    def mover_robot_a_destino(self, goal_pose):
        speed_command_list = []
        x = goal_pose[0]
        y = goal_pose[1]
        angle = goal_pose[2]
        seconds = (x - self.predict_pose[0]) / self.vel
        if abs(x - self.predict_pose[0]) >= 0.5 :
            if seconds >= 0.0:
                speed_command_list.append((self.vel, 0.0, seconds))
            else:
                speed_command_list.append((-1 * self.vel, 0.0, abs(seconds)))
        seconds = (y - self.predict_pose[1]) / self.vel
        if abs(y - self.predict_pose[1]) >= 0.5 :
            speed_command_list.append((0.0, self.turn_vel, np.deg2rad(90.0) + self.offset))
            if seconds >= 0.0:
                speed_command_list.append((self.vel, 0.0, seconds))
            else:
                speed_command_list.append((-1 * self.vel, 0.0, abs(seconds)))
            speed_command_list.append((0.0, -1 * self.turn_vel,  np.deg2rad(90.0) + self.offset))
        self.aplicar_velocidad(speed_command_list)
        
    def accion_mover_cb(self, coordenates: PoseArray):
        for coord in coordenates.poses:
            x = coord.position.x
            y = coord.position.y
            angle = coord.position.z
            self.mover_robot_a_destino((x, y, angle))

    def start_moving_node(self, coordenates: PoseArray):
        self.thread_movement = Thread(target=self.accion_mover_cb, args=(coordenates,))
        self.thread_movement.start()

    def read_odometry(self, data: Odometry):
        self.odo_x = data.pose.pose.position.x
        self.odo_y = data.pose.pose.position.y
        self.odo_z = data.pose.pose.position.z
        self.pos_odo_active = True
        
    def read_real(self, data: Pose):
        self.real_x = data.position.x
        self.real_y = data.position.y
        self.real_z = data.position.z
        self.pos_real_active = True

    def say_pos(self):
        if self.pos_odo_active:
            self.get_logger().info( 'odometry pos (%f, %f, %f)' % (self.odo_x, self.odo_y, self.odo_z) )
        if self.pos_real_active:
            self.get_logger().info( 'real pos (%f, %f, %f)' % (self.real_x, self.real_y, self.real_z) )
        self.get_logger().info( 'predicted pose (%f, %f, %f)' % (self.predict_pose[0], self.predict_pose[1], self.predict_pose[2]) )

def main(args=None):
    rclpy.init(args=args) #? Inicializa ROS
    node = Movement_Node() #? Instancia de un nodo de clase MyNode
    rclpy.spin(node) #? Permite ejecución continua de un nodo
    rclpy.shutdown() #? Cierra el nodo al terminar la ejecución
    
if __name__ == "__main__":
    main()