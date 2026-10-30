#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3" ^
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class Seguidor_obj(Node): 

    def __init__(self):
        super().__init__("seguidor_obj")
        self.init_communications()
        self.setup_parameters()

    def setup_parameters(self):
        self.max_v = 0.2
        self.z = 1.0
        self.x = 0
        self.kp = 1.0
        self.max_w = 0.2

    def init_communications(self):
        self.subscription_goal = self.create_subscription(Point, "blue_square_position", self.seguir, 1)
        self.cmd_vel_mux_pub = self.create_publisher( Twist, '/commands/velocity', 10 )

    def seguir(self, msg: Point):
        self.x = msg.x
        self.z = msg.z
        speed = Twist()
        if self.z != 1.0:
            actuation = -self.kp * self.x
            speed.angular.z = max(-self.max_w, min(self.max_w, actuation))
            if abs(self.x) <= 50:
                vel = self.max_v
            else:
                vel = 0.0
            speed.linear.x = vel
            self.get_logger().info( 'publishing speed (%f, %f)' % (vel, max(-self.max_w, min(self.max_w, actuation))) )
            self.cmd_vel_mux_pub.publish(speed)
        else:
            speed.angular.z = 0.0
            speed.linear.x = 0.0
            self.get_logger().info( 'publishing speed (%f, %f)' % (0.0, 0.0) )
            self.cmd_vel_mux_pub.publish(speed)

def main(args=None):
    rclpy.init(args=args) #? Inicializa ROS
    node = Seguidor_obj() #? Instancia de un nodo de clase MyNode
    rclpy.spin(node) #? Permite ejecución continua de un nodo
    rclpy.shutdown() #? Cierra el nodo al terminar la ejecución
    
if __name__ == "__main__":
    main()