#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np
from parametros import DIST_CAMBIO, DIST_ZANAHORIA
from geometry_msgs.msg import Point
import math

class FollowTheCarrot(Node):

    def __init__(self):
        super().__init__('FollowTheCarrot')
        # Nos suscribimos al tópico para recibir la ruta
        self.suscripcion_ruta = self.create_subscription(Path, 'path', self.recibir_ruta, 10)    
        

        # Queue FIFO para almacenar las rutas
        self.queue_ruta = []

        # Nos suscribimos a la odometría
        self.suscripcion_odometria = self.create_subscription( Odometry, '/odom', self.recibir_odometria, 10 )
        # Definición de la variable que almacena las coordenadas de la zanahoria
        self.zanahoria = None

        # Publicamos en el tópico para que el controlador lea el mensaje
        self.publicador_setpoint_state = self.create_publisher( Point, 'setpoint_state', 1 )
    
    def recibir_ruta(self, path_msg):
        # Callback para recibir las rutas
        for pose_stamped  in path_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.queue_ruta.append([x,y])
        
        #self.get_logger().info("Llegó la ruta")


    def colocar_zanahoria(self, x_robot, y_robot, x_obj, y_obj):
        # Función que calcula la coordenada entre (x,y) robot y el par (x,y) objetivo
        # a una distancia DIS_ZANAHORIa del objetivo
        dx = x_robot - x_obj
        dy = y_robot - y_obj
        distancia = math.hypot(dx, dy)

        if distancia == 0 or DIST_ZANAHORIA > distancia:
            # Ya estamos en el objetivo o el robot está demasiado cerca
            return x_robot, y_robot

        # Vector unitario desde el objetivo hacia el robot
        ux = dx / distancia
        uy = dy / distancia

        # Punto a DIST_ZAN desde el objetivo hacia el robot
        x_zanahoria = x_obj + ux * DIST_ZANAHORIA
        y_zanahoria = y_obj + uy * DIST_ZANAHORIA
        return x_zanahoria, y_zanahoria
        
    def calcular_angulo_zanahoria(self,x_robot , y_robot, x_zanahoria, y_zanahoria):
        dx = x_zanahoria - x_robot
        dy = y_zanahoria - y_robot
        # La función da en términos de radianes entre -π y π
        angulo = math.atan2(dy, dx)  
        return angulo
            

    def recibir_odometria(self, msg_odom):
        if self.queue_ruta == []:
            #self.get_logger().info("No hay nada que revisar")
            return


        x_robot = msg_odom.pose.pose.position.x
        y_robot = msg_odom.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion( ( msg_odom.pose.pose.orientation.x,
                                                    msg_odom.pose.pose.orientation.y,
                                                    msg_odom.pose.pose.orientation.z,
                                                    msg_odom.pose.pose.orientation.w ) )
        angulo_robot = yaw
        #self.get_logger().info(f"{angulo_robot}")
        if self.zanahoria == None:
            # Caso inicial
            x_obj, y_obj = self.queue_ruta.pop(0)
            self.zanahoria = self.colocar_zanahoria(x_robot, y_robot, x_obj, y_obj)
            #self.get_logger().info(f"Primera zanahoria es {self.zanahoria}")
        elif np.sqrt((x_robot-self.zanahoria[0])**2 + (y_robot-self.zanahoria[1])**2) <= DIST_CAMBIO:
            # Estamos muuy cerca de la zanahoria. Reemplazamos la zanahoria por otra
            x_obj, y_obj = self.queue_ruta.pop(0)
            self.zanahoria = self.colocar_zanahoria(x_robot, y_robot, x_obj, y_obj)
        
        #self.get_logger().info(f"Nuestra zanahora {self.zanahoria} y esta a distancia {np.sqrt((x_robot-self.zanahoria[0])**2 + (y_robot-self.zanahoria[1])**2)}")
        angulo_zanahoria = self.calcular_angulo_zanahoria(x_robot, y_robot, self.zanahoria[0], self.zanahoria[1])

        self.publicar_setpoint(angulo_robot, angulo_zanahoria)
        
    def publicar_setpoint(self, angulo_robot, angulo_zanahoria):
        # Publicamos en el tópico para que el controlador lea el mensaje
        msg = Point()
        msg.x = angulo_zanahoria
        msg.y = angulo_robot
        self.publicador_setpoint_state.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheCarrot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                             