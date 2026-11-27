#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3" ^
import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Twist

class Client_node(Node): 

    def __init__(self):
        super().__init__("client_node")
        self.init_communications()
        self.setup_parameters()

    def setup_parameters(self):
        self.timer = self.create_timer(1.0, self.start)

    def init_communications(self):
        self.publisher = self.create_publisher(Empty, "/explore", 10 ) #cambiar tipo de msg
        self.subscription_goal = self.create_subscription(Pose, "/displacement", self.recive_pos, 1)
        self.started = False
        self.retry_timer = self.create_timer(0.5, self.kickstart)

    def recive_pos(self, msg):
        self.publisher.publish(Empty())

    def start(self):
        self.timer.cancel()
        self.publisher.publish(Empty())

    def kickstart(self):
        if not self.started:
            self.publisher.publish(Empty())

    def recive_pos(self, msg):
        if not self.started:
            self.started = True
            self.retry_timer.cancel()
        self.publisher.publish(Empty())

def main(args=None):
    rclpy.init(args=args) #? Inicializa ROS
    node = Client_node() #? Instancia de un nodo de clase MyNode
    rclpy.spin(node) #? Permite ejecución continua de un nodo
    rclpy.shutdown() #? Cierra el nodo al terminar la ejecución
    
if __name__ == "__main__":
    main()