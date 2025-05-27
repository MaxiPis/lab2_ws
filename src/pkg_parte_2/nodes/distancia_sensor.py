#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import numpy as np

#De momento asumire que el simulador usara el mismo topico de comunicacion que el del laboratorio 1#

# Este nodo debe suscribirse al kinect y 
# obtener la distancia a la pared izquierda y derecha

# El nodo debe publicar en el tópico /dl_dr (así lo llamé)
# El msg es del tipo Float64MultiArray
# dl=  Distancia izuiqrrda
# dr=  Distancia derecha
# Se publica como (dl, dr)

# Publicamos todo el tiempo


class DistanceSensor(Node):
    def __init__(self):
        super().__init__('distance_sensor')
        self.get_logger().info('Distance Sensor Node has been started')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Vector3, '/dl_dr', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    distance_sensor = DistanceSensor()
    rclpy.spin(distance_sensor)
    distance_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
