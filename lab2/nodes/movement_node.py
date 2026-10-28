#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import numpy as np
from threading import Thread
from tf_transformations import euler_from_quaternion
import time

def normalizar(angulo):
    # basicamente escoge el angulo mas corto en el que hay que rotar
    return np.arctan2(np.sin(angulo), np.cos(angulo))

class Movement_Node(Node):

    def __init__(self):
        super().__init__("movement_node")
        self.init_communications()
        self.setup_parameters()
    
    def setup_parameters(self):
        self.odo_x = 0.0
        self.odo_y = 0.0
        self.odo_yaw = 0.0
        self.pos_odo_active = False
        self.mode = None # para lineal: "lin", para angular: "ang"

    def init_communications(self):
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.goal_sub = self.create_subscription(PoseArray, "goal_list", self.start_moving_node, 1)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.read_odometry, 1)

        # Comunicaciones con controlador desplazamieneto lineal
        self.setpoint_pub_lin = self.create_publisher(Float64, "lin/setpoint", 1)
        self.state_pub_lin = self.create_publisher(Float64, "lin/state", 1)
        self.control_sub_lin = self.create_subscription(Float64, "lin/control_effort", self.lin_effort_cb, 1)
        
        # Comunicaciones con controlador desplazamieneto rotacional
        self.setpoint_pub_ang = self.create_publisher(Float64, "ang/setpoint", 1)
        self.state_pub_ang = self.create_publisher(Float64, "ang/state", 1)
        self.control_sub_ang = self.create_subscription(Float64, "ang/control_effort", self.ang_effort_cb, 1)

    def aplicar_velocidad(self, displacement_list):
        # Esperamos a que termine el desplazamiento actual
        while not self.pos_odo_active:
            time.sleep(0.1)

        for linear_disp, rotational_disp in displacement_list:
            if linear_disp != 0.0 and rotational_disp == 0.0: # Desplazamiento lineal
                mode, goal = "lin", linear_disp
            elif rotational_disp != 0.0 and linear_disp == 0.0: # Desplazamiento rotacional
                mode, goal = "ang", rotational_disp
            else:
                continue

            x0, y0, th0 = self.odo_x, self.odo_y, self.odo_yaw
            self.mode = mode

            # Publicamos el punto de referencia al controlador:
            msg = Float64()
            msg.data = goal
            if mode == "lin":
                self.setpoint_pub_lin.publish(msg)
            elif mode == "ang":
                self.setpoint_pub_ang.publish(msg)

            # Bucle de control y retroalmentación del state
            error = goal
            while abs(error) > 0.01: # 0.01 es el error que max toleraremos, cambiar si es necesario
                # En el caso lineal calculamos distancia de pitagoras
                if mode == "lin":
                    dx = self.odo_x - x0
                    dy = self.odo_y - y0
                    state = np.hypot(dx, dy)

                    msg = Float64()
                    msg.data = state
                    self.state_pub_lin.publish(msg)
                    error = goal - state

                # En el caso rotacional calculamos la distancia angular más corta
                elif mode == "ang":
                    angulo_ruta = self.odo_yaw - th0
                    state = normalizar(angulo_ruta)

                    msg = Float64()
                    msg.data = state
                    self.state_pub_ang.publish(msg)
                    error = normalizar(goal - state)

                time.sleep(0.05) # Frecuencia de publicción del estado, evitamos solapar
            
            self.mode = None
            self.cmd_pub.publish(Twist())

    def mover_robot_a_destino(self, goal_pose):
        x, y, theta = goal_pose
        displacement_list = [(x, 0.0), (0.0, theta), (y, 0.0)]
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

    # Retroalimentación
    def read_odometry(self, data: Odometry):
        self.odo_x = data.pose.pose.position.x
        self.odo_y = data.pose.pose.position.y
        self.odo_yaw = euler_from_quaternion(
            (data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w))[2]
        self.pos_odo_active = True

    # Actuación
    def lin_effort_cb(self, msg: Float64):
        if self.mode == "lin":
            cmd = Twist()
            cmd.linear.x = msg.data
            self.cmd_pub.publish(cmd)

    def ang_effort_cb(self, msg: Float64):
        if self.mode == "ang":
            cmd = Twist()
            cmd.angular.z = msg.data
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Movement_Node()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()