#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import queue
from parametros import RUTA_ARCHIVO
import os

class NodoError(Node):
    def __init__(self):
        super().__init__('line_follower_error_node')

        self.queue_ruta = []
        self.error = 0
        self.numero_datos = 0

        #dir_route = os.path.dirname(os.path.abspath(__file__)) #Consigue la ruta del directorio actual
        #coordenadas_file_path = os.path.join(dir_route, RUTA_ARCHIVO)
        coordenadas_file_path = f"mapas/{RUTA_ARCHIVO}"
        #self.get_logger().info(f"{coordenadas_file_path}")
        with open(coordenadas_file_path, 'r') as archivo:
            for linea in archivo:
                partes = linea.strip().split()
                x , y = partes[0].split(",")
                x, y = float(x), float(y)
                self.queue_ruta.append((x,y))
        
    
        # Suscribirse a la odometría
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("Nodo iniciado y esperando datos de odometría...")



    def odom_callback(self, msg: Odometry):
        punto_deseado = self.queue_ruta.pop(0)

        
        pos = msg.pose.pose.position
        odom_point = (pos.x, pos.y)

        # Calcular error cuadrático
        error_x = odom_point[0] - punto_deseado[0]
        error_y = odom_point[1] - punto_deseado[1]
        error_cuadrado = error_x**2 + error_y**2
        self.error += error_cuadrado
        self.numero_datos +=1
        self.get_logger().info(f"Error {self.error }, Datos {self.numero_datos}")

        #self.get_logger().info(
        #    f"Odometría: {odom_point}, Deseado: {punto_deseado}, Error²: {error_cuadrado:.6f}"
        #)

def main(args=None):
    rclpy.init(args=args)
    node = NodoError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
