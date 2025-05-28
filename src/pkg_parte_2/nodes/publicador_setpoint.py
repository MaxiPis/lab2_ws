#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

# La idea de este nodo es que inicialice el setpoint y por tanto el controlador_pasillo
# Envía un mensaje a setpoint que será el deseo, en este caso, es que sea 0
# Me hace sentido que sólo el controlador funcione si es que hay un deseo, por tanto publica constantemente


class SetpointPublisher(Node):
    def __init__(self):
        super().__init__('publicador_setpoint')
        self.get_logger().info('Iniciando publicador de setpoint...')
        # Inicializar el setpoint deseado
        self.setpoint_wish = 0.0

        # Crear publicador para el setpoint
        self.setpoint_pub = self.create_publisher(Float64, '/setpoint', 1)
        self.timer = self.create_timer(0.2, self.publish_setpoint)

    def publish_setpoint(self):
        msg = Float64()
        msg.data = self.setpoint_wish
        self.setpoint_pub.publish(msg)
        self.get_logger().info(f'[SETPOINT] Publicando setpoint: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SetpointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
