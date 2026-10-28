#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3" ^
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class Detector_obj(Node): 

    def __init__(self):
        super().__init__("detector_obj")
        self.init_communications()
        self.setup_parameters()

    def setup_parameters(self):
        self.cx = None
        self.cy = None

    def init_communications(self):
        self.publisher = self.create_publisher( Point, "blue_square_position", 10 ) #cambiar tipo de msg
        self.subscription_goal = self.create_subscription(Image, "/camera/rgb/image_color", self.img_callback, 1)
        self.bridge = CvBridge()
        
    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w = img.shape[:2]
        mask = cv2.inRange(hsv_img, (100, 100, 50), (130, 255, 255))
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        if not cnts:
            self.cx = None
            self.cy = None
        else:
            square = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(square) < 500:
                self.cx = None
                self.cy = None
            else:
                M = cv2.moments(square)
                self.cx = (M["m10"] / M["m00"]) - w // 2
                self.cy = (M["m01"] / M["m00"]) - h // 2 
        self.send_info()

    def send_info(self):
        if self.cx is not None and self.cy is not None:
            self.publisher.publish(Point(x = float(self.cx), y = float(self.cy), z = 0.0))
        else:
            self.publisher.publish(Point(x = 0.0, y = 0.0, z = 1.0))

def main(args=None):
    rclpy.init(args=args) #? Inicializa ROS
    node = Detector_obj() #? Instancia de un nodo de clase MyNode
    rclpy.spin(node) #? Permite ejecución continua de un nodo
    rclpy.shutdown() #? Cierra el nodo al terminar la ejecución
    
if __name__ == "__main__":
    main()
