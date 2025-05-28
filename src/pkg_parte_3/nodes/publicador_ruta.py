#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import os
from parametros import RUTA_ARCHIVO

class PublicadorRuta(Node):

    def __init__(self):
        super().__init__('publicador_ruta')

        self.publisher = self.create_publisher(Path, 'path', 10)    
        self.leer_ruta()

    def leer_ruta(self):
        # Instanciamos la estructura de datos
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        dir_route = os.path.dirname(os.path.abspath(__file__)) #Consigue la ruta del directorio actual
        coordenadas_file_path = os.path.join(dir_route, RUTA_ARCHIVO)

        with open(coordenadas_file_path, 'r') as archivo:
            for linea in archivo:
                partes = linea.strip().split()
                x , y = partes[0].split(",")
                x, y = float(x), float(y)

                # Almacenamos en cada variable 
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                path_msg.poses.append(pose)
        self.publicar_ruta(path_msg)
    
    def publicar_ruta(self, path_msg):
        #self.get_logger().info("Publisher Ruta: Listo")
        self.publisher.publish(path_msg)

def main(args=None):

    rclpy.init(args=args)
    node = PublicadorRuta()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()
