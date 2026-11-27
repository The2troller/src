#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

class Client_node(Node): 

    def __init__(self):
        super().__init__("client_node")
        self.started = False
        self.publisher = self.create_publisher(Empty, "/explore", 10)
        self.subscription_goal = self.create_subscription(Pose, "/displacement", self.recive_pos, 10)
        self.retry_timer = self.create_timer(0.5, self.kickstart)

    def kickstart(self):
        if not self.started:
            self.publisher.publish(Empty())

    def recive_pos(self, msg):
        if not self.started:
            self.started = True
            self.retry_timer.cancel()
        self.publisher.publish(Empty())

def main(args=None):
    rclpy.init(args=args)
    node = Client_node()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()