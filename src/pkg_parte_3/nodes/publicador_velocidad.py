#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from parametros import VEL_LINEAL

class PublicadorVelocidad(Node):

    def __init__(self):
        super().__init__('PublicadorVelocidad')
        # Cambiar topico y la estructura de datos recibida
        self.suscripcion = self.create_subscription(Float64,'respuesta_control',self.aplicar_velocidad,1)        
        
        #Publicador de velocidades
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel_mux/input/navigation',10)

    def aplicar_velocidad(self, msg):
        #self.get_logger().info(f"Publisher twsit: {msg.data}")
        

        # Instanciamos la EDD
        twist = Twist()
        # Asignamos la velocidad lineal
        twist.linear.x = VEL_LINEAL
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        # Asignamos la velocidad angular
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = msg.data

        # Publicamos en el topico
        self.publisher_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PublicadorVelocidad()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                             