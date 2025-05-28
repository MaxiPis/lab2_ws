#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# Recibe la velocidad angular del controlador_pasillo

# Crea el mensaje de tipo twist y agrega en el mensaje la velocidad
# lineal y la velocidad angular

# Publca en cmd_vel_mux

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('publicador_velocidad')
        self.get_logger().info('Iniciando publicador de velocidad...')

        self.linear_velocity = 0.2  # Velocidad lineal, constante en 0.2 m/s
        # Subscribers
        self.angular_velocity_sub = self.create_subscription(
            Float64,
            'control_effort',
            self.angular_velocity_callback,
            1
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_mux/input/navigation', 1)

    def angular_velocity_callback(self, msg):
        self.get_logger().info(f'[VELOCIDAD] Recibida velocidad angular: {msg.data}')

        # Crear mensaje Twist
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = msg.data
        self.get_logger().info(f'[VELOCIDAD] Publicando cmd_vel: {twist_msg}')
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
