#! /usr/bin/env python3
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PublicadorVel(Node):

    def __init__(self):
        super().__init__('PublicadorVelocidad')
        # Suscripción al esfuerzo de control
        self.subscription_lineal = self.create_subscription(Float64,'control_lienal',self.aplicar_velocidad_lineal,10)
        self.subscription_angular = self.create_subscription(Float64,'control_angular',self.aplicar_velocidad_angular,10)
        # Publicador de velocidades
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel_mux/input/navigation',10)

    def aplicar_velocidad_lineal(self, msg):
        # Instancia el mensaje Twist
        twist = Twist()
        twist.linear.x = msg.data
        # Publica en el tópico /cmd_vel_mux/input/navigation
        self.publisher_cmd_vel.publish(twist)
    def aplicar_velocidad_angular(self, msg):
        # Instancia el mensaje Twist
        twist = Twist()
        twist.angular.z = msg.data
        # Publica en el tópico /cmd_vel_mux/input/navigation
        self.publisher_cmd_vel.publish(twist)

def main(args=None):
    print("publicador vel node funcionando")
    rclpy.init(args=args)
    node = PublicadorVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                             