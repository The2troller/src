#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from threading import Thread

class Movement_Node(Node):

    def __init__(self):
        super().__init__("movement_node")
        self.init_communications()
        self.setup_parameters()
    
    def setup_parameters(self):
        self.velocity = Twist()
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.pos_odo_active = False

    def init_communications(self):
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_sub = self.create_subscription(PoseArray, "goal_list", self.start_moving_node, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.read_odometry, 10)

        # Publicaciones de referencia y estado al controlador
        self.setpoint_pub = self.create_publisher(Float64, "setpoint", 10)
        self.state_pub = self.create_publisher(Float64, "state", 10)
        # Obtenemos la retroalimentación del controlador
        self.control_sub = self.create_subscription(Float64, "control_effort", self.control_callback, 10)

    def aplicar_velocidad(self, displacement_list):
        for linear_disp, rotational_disp in displacement_list:
            if abs(linear_disp) > 0.001:
                objetivo = linear_disp
                tipo = "LINEAL"
            elif abs(rotational_disp) > 0.001:
                objetivo = rotational_disp
                tipo = "ANGULAR"
            else:
                continue
        
        msg = Float64()
        msg.data = float(objetivo)
        self.get_logger().info(f"Enviando setpoint {tipo}: {objetivo:.2f}")
        self.setpoint_pub.publish(msg)
        
    def mover_robot_a_destino(self, goal_pose):
        x, y, theta = goal_pose
        displacement_list = [(x, 0.0), (0.0, theta), (y, 0.0)]

        self.get_logger().info(f"Se mueve ({x:.2f}, {y:.2f}: {displacement_list}")
        self.aplicar_velocidad(displacement_list)

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

def main(args=None):
    rclpy.init(args=args)
    node = Movement_Node()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()