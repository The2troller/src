#!/usr/bin/env python3
# librerias que vamos a utilizar

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Teleop_Node(Node):

    def __init__(self):
        super().__init__("teleop")
        self.init_communications()

    def init_communications(self):
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def send_velocity(self, lineal, angular):
        velocity = Twist()
        velocity.linear.x = lineal # avanzar positivo retroceder negativo
        velocity.angular.z = angular # positivo izquierda negativo derecha
        self.publisher.publish(velocity)

def get_key():
    settings = termios.tcgetattr(sys.stdin) # guarda la info de la terminal
    tty.setraw(sys.stdin.fileno()) # cambia a modo raw
    key = sys.stdin.read(1) # espera a que aprietes teclas
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # vuelve la terminal a la normalidad
    return key

def main():
    rclpy.init()
    node = Teleop_Node()
    print("i adelante | j atras | a/s girar | q/w movimiento curvo | espacio detenerse | x salir")

    while True:
        key = get_key()
        if key == 'x' or key == '\x03':
            break
        # basicamente todas las posibilidades
        if key == 'i':
            lineal, angular = 0.2, 0.0
        elif key == 'j':
            lineal, angular = -0.2, 0.0
        elif key == 'a':
            lineal, angular = 0.0, 1.0
        elif key == 's':
            lineal, angular = 0.0, -1.0
        elif key == 'q':
            lineal, angular = 0.2, 1.0
        elif key == 'w':
            lineal, angular = 0.2, -1.0
        else:
            lineal, angular = 0.0, 0.0 # frenar en caso de cualquier otra cosa
        node.send_velocity(lineal, angular)

    node.send_velocity(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown() # al salir apaga todo ydeja de mover al robot


if __name__ == "__main__":
    main()