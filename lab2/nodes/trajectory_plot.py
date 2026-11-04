#!/usr/bin/env python3

import csv
import math
import os
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ploteo(Node):

    def __init__(self):   # literalmente definimos un nuevo nodo llamado ploteo
        super().__init__("ploteo")
        self.init_communications()
        self.setup_parameters()

    def setup_parameters(self):
        self.declare_parameter("salida", "trayectoria")   # creamos estos parametros

        self.plan_x = []        # trayectoria que deberia ocurrir
        self.plan_y = []
        self.real_x = []        # trayectoria que recorre el robot
        self.real_y = []

        plt.ion()               # pantalla ventana se actualiza sin bloquear !!
        self.fig, self.ax = plt.subplots()  # definimos para que tenga todo lo necesario en cada eje
        self.line_plan, = self.ax.plot([], [], "b--", label="Trayectoria debida")
        self.line_real, = self.ax.plot([], [], "r-", label="Trayectoria real con PID")
        self.ax.set_xlabel("x en metros")
        self.ax.set_ylabel("y en metros")
        self.ax.set_title("Recorrido real vs Ideal")
        self.ax.set_aspect("equal", adjustable="datalim") # caja exactamente de 1 metro visualmente para ambos lados
        self.ax.grid(True) # con cuadricula
        self.ax.legend(loc="best")  # basicamente mostrar el cuadrado

    def init_communications(self):    # basicamente debemos escuchar la pos real del robot y la que queremos seguir
        self.path_sub = self.create_subscription(Path, "nav_plan", self.path_cb, 1)
        self.real_sub = self.create_subscription(Pose, "/real_pose", self.real_cb, 1)

    def path_cb(self, msg: Path): # listas que contiene la pos del real vs la que ya tiene el camino correcto a recorrer
        self.plan_x = [p.pose.position.x for p in msg.poses]
        self.plan_y = [p.pose.position.y for p in msg.poses]
        self.real_x = []
        self.real_y = []

    def real_cb(self, msg: Pose): # basicamente cuando llega el mensaje de la pos real en caso de que sea mayor a 0.002 se guarde para despues graficarlo
        x = msg.position.x
        y = msg.position.y
        if len(self.real_x) == 0 or math.hypot(x - self.real_x[-1], y - self.real_y[-1]) > 0.002:
            self.real_x.append(x)
            self.real_y.append(y)

    def actualizar(self):  
        self.line_plan.set_data(self.plan_x, self.plan_y)
        self.line_real.set_data(self.real_x, self.real_y)
        self.ax.relim()  # mira que tan grande son los datos para luego
        self.ax.autoscale_view()   # re escalar en caso de que sea necesario
        plt.pause(0.05)

    def guardar_csv(self, ruta, lista_x, lista_y):
        with open(ruta, "w", newline="") as archivo:
            escritor = csv.writer(archivo)
            escritor.writerow(["x", "y"])
            for i in range(len(lista_x)):
                x = lista_x[i]
                y = lista_y[i]
                escritor.writerow([x, y])       # una fila por punto

    def guardar(self):
        # funcion que se llama cuando guardamos al final
        hay_trayectoria_original = len(self.plan_x) > 0
        hay_recorrido_robot = len(self.real_x) > 0
        if not hay_trayectoria_original and not hay_recorrido_robot:
            self.get_logger().info("No datos")
            return

        nombre_archivo = self.get_parameter("salida").value
        nombre = os.path.splitext(nombre_archivo)[0] # nos quedamos con solo en nombre sin el .txt
        carpeta = "resultados"
        os.makedirs(carpeta, exist_ok=True)   # crear carpeta en caso de no existir
        ruta_plan = f"{carpeta}/{nombre}_plan.csv"      # trayectoria original
        ruta_real = f"{carpeta}/{nombre}_real.csv"      # recorrido del robot
        ruta_imagen = f"{carpeta}/{nombre}.png"         # foto del grafico
        self.guardar_csv(ruta_plan, self.plan_x, self.plan_y) # guardamos los puntos
        self.guardar_csv(ruta_real, self.real_x, self.real_y)
        self.fig.savefig(ruta_imagen, dpi=150) # guardamos la foto final


def main(args=None):
    rclpy.init(args=args)
    node = ploteo()
    try:
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            node.actualizar()
    except KeyboardInterrupt:  # literalmente le agrego el anti error porque se cierra solo
        pass
    node.guardar()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()