#!/usr/bin/env python3
#? ^ Le dice a bash "ejecuta esto con python3" ^
import rclpy
from rclpy.node import Node


from geometry_msgs.msg import PoseArray, Pose

class Pose_Loader(Node): 

    def __init__(self):
        super().__init__("pose_loader") # le cambie el nombre de "node_name a ..."
        self.init_communications()
        self.setup_parameters()
        self.timer = self.create_timer(2.0, self.start_send) # rutas relativas ojo

    def start_send(self):
        self.timer.cancel()
        self.send_data("src/lab1/text_files/box.txt") # rutas relativas ojo

    def setup_parameters(self):
        self.sending_data = PoseArray()

    def send_data(self, path):
        data_array = []
        with open(path, "r") as file:
            self.file = file.readlines()
            for i in range(len(self.file)):
                self.file[i] = self.file[i].strip().split(",")
                coord = Pose()
                coord.position.x = float(self.file[i][0])
                coord.position.y = float(self.file[i][1]) # arreglo z ya que es el angulo
                coord.position.z = float(self.file[i][2])
                data_array.append(coord)
        self.sending_data.poses = data_array
        self.publisher.publish(self.sending_data)

    def init_communications(self):
        self.publisher = self.create_publisher( PoseArray, "goal_list", 10 )
        


def main(args=None):
    rclpy.init(args=args) #? Inicializa ROS
    node = Pose_Loader() #? Instancia de un nodo de clase MyNode
    rclpy.spin(node) #? Permite ejecución continua de un nodo
    rclpy.shutdown() #? Cierra el nodo al terminar la ejecución
    
if __name__ == "__main__":
    main()