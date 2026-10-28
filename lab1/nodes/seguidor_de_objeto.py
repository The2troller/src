#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3" ^
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

class Seguidor_obj(Node): 

    def __init__(self):
        super().__init__("seguidor_obj")
        self.init_communications()
        self.setup_parameters()

    def setup_parameters(self):
        pass

    def init_communications(self):
        
        self.subscription_goal = self.create_subscription(Point, "blue_square_position", self.callback, 1)
        
    def callback():
        pass

def main(args=None):
    rclpy.init(args=args) #? Inicializa ROS
    node = Seguidor_obj() #? Instancia de un nodo de clase MyNode
    rclpy.spin(node) #? Permite ejecución continua de un nodo
    rclpy.shutdown() #? Cierra el nodo al terminar la ejecución
    
if __name__ == "__main__":
    main()