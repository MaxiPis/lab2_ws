#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np

#De momento asumire que el simulador usara el mismo topico de comunicacion que el del laboratorio 1#

#Max#
# Este nodo debe suscribirse al kinect y 
# obtener la distancia a la pared izquierda y derecha

# El nodo debe publicar en el tópico /dl_dr (así lo llamé)
# El msg es del tipo Float64MultiArray
# dl=  Distancia izuiqrrda
# dr=  Distancia derecha
# Se publica como (dl, dr)


#Quimi#
#Dado que el simulador nos envia la info por su topico, defini que este nodo sea el que reciba la info del sensor y envie el mensaje a controlador pasillo con el resultado de la diferencia de las distancias, es decir este nodo se encarga de recibir la imagen del sensor, calcular la distancia a las paredes izquierda y derecha y publicar el mensaje en el tópico /dl_dr como un msg Float64, de manera que controlador pasillo se encarga del resto del hilo de flujo.

# Publicamos todo el tiempo


class DistanceSensor(Node):
    def __init__(self):
        super().__init__('distance_sensor')
        self.get_logger().info('El sensor de distancia ha sido iniciado')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Float64, '/dl_dr', 10)
        self.distance_msg = Float64()

        self.bridge = CvBridge()
        self.current_cv_depth_distance = None

    def listener_callback(self, msg):
        """Esta funcion sera la callback para el topico que recibe desde el kinect, debe obtener la profundidad y calcular la diferencia de distancia entre las paredes izquierda y derecha."""
        #self.get_logger().info('Received image from camera')
        try:
            self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') # Mantiene el tipo de datos original, en caso de que sea un tipo de imagen diferente
            if msg.encoding == '16UC1':
                self.current_cv_depth_image = self.current_cv_depth_image.astype(np.float32) / 1000.0  # Convertir a metros, en caso de que venga en milimetros

            # Separar matriz en regiones izquierda y derecha
            height, width = self.current_cv_depth_image.shape
            left_region = self.current_cv_depth_image[:, :width // 3]
            #middle_region = self.current_cv_depth_image[:, width // 3:2 * width // 3]
            right_region = self.current_cv_depth_image[:, width // 3:]
            # ESto dividira la imagen en 3 mitades utilizaremos solo las laterales

            # Calcular la distancia promedio en cada región
            # No creo que hayan NaNs en la imagen dada que es simulada, pero encontre esta funcion que calcula el promedio ignorando los NaNs, por lo que encuentro que es una buena idea usarla
            left_mean_distance = np.nanmean(left_region)
            right_mean_distance = np.nanmean(right_region)

            # Calcular la diferencia de distancia entre las paredes izquierda y derecha
            self.distance_msg.data = right_mean_distance - left_mean_distance
            #Si es negativo significa que estamos mas cerca de la pared izquierda, si es positivo estamos mas cerca de la pared derecha, si es 0, estamos en medio#
            self.publish_distance(self.distance_msg)
        except Exception as e:
            self.get_logger().error(f'Error al obtener profundidad u obtener los datos: {e}')
    
    def publish_distance(self, msg):
        """Publica la distancia calculada en el tópico /dl_dr."""
        self.get_logger().info('Publishing distance to /dl_dr')
        self.get_logger().info(f'Distance: {self.distance_msg.data}')
        self.publisher.publish(self.distance_msg)

def main(args=None):
    rclpy.init(args=args)
    distance_sensor = DistanceSensor()
    rclpy.spin(distance_sensor)
    distance_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
