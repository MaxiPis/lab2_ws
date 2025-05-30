#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64,Float64MultiArray
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class Controlador_P(Node):
    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.1):
        super().__init__('controlador_P')
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.setpoint = None
        self.state = None
        self.tipo_control = None

        self.pose_pub = self.create_publisher(Float64MultiArray, '/pose_actual', 10)
        self.actuation_pub_lineal = self.create_publisher(Float64, 'control_lienal', 10)
        self.actuation_pub_angular = self.create_publisher(Float64, 'control_angular', 10)

        self.subscripcion_tupla = self.create_subscription(Float64MultiArray, '/setpoint_P', self.setpoint_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        self.u_k = 0.0
        self.u_k_1 = 0.0
        self.error = 0.0
        self.error_1 = 0.0
        self.error_2 = 0.0


    def setpoint_cb(self, msg):
        self.parte_lineal = msg.data[0]
        self.parte_angular = msg.data[1]

        if self.parte_lineal != 0 and self.parte_angular == 0:
            self.tipo_control = 'controlador_lineal'
            self.setpoint = (self.parte_lineal, 0.0)
        elif self.parte_lineal == 0 and self.parte_angular != 0:
            self.tipo_control = 'controlador_angular'
            self.setpoint = (0.0, 0.0, self.parte_angular)
        else:
            self.get_logger().warn(f"Setpoint inválido: {msg.data}")
            return

        self.get_logger().info(f'Setpoint recibido: {self.setpoint} ({self.tipo_control})')

    def odometry_callback(self, odom):
        if self.setpoint is None:
            return

        if self.tipo_control == "controlador_lineal":
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            self.state = (x, y)

            distancia_actual = (x**2 + y**2)**0.5
            distancia_objetivo = self.setpoint[0]
            error = abs(distancia_objetivo - distancia_actual)
            self.error = distancia_objetivo - distancia_actual

            if error < 0.01:
                msg = Float64MultiArray()
                msg.data = [x, y, 0.0]
                self.get_logger().info(f"[ACK] Llegó al destino: {msg.data}")
                self.pose_pub.publish(msg)
                self.setpoint = None
                return

        elif self.tipo_control == "controlador_angular":
            q = odom.pose.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.state = yaw

            yaw_deseado = self.setpoint[2]
            error = abs(yaw - yaw_deseado)
            self.error = yaw_deseado - yaw
            #self.get_logger().info(f'Orientación actual (yaw): {yaw:.4f} rad')
            if error < 0.01:
                msg = Float64MultiArray()
                msg.data = [0.0, 0.0, yaw]
                self.get_logger().info(f"[ACK] Llegó al destino: {msg.data}")
                self.pose_pub.publish(msg)
                self.setpoint = None
                return

        self.state_cb()

    def state_cb(self):
        if self.tipo_control == "controlador_lineal":
            setpoint_tuple = self.setpoint
            self.error = ((setpoint_tuple[0] - self.state[0])**2 + (setpoint_tuple[1] - self.state[1])**2)**0.5

        elif self.tipo_control == "controlador_angular":
            #self.get_logger().info(f"aquiii: {self.setpoint[2],self.state,}")
            yaw_deseado = self.setpoint[2]
            self.error = yaw_deseado - self.state

        else:
            self.get_logger().warn("Tipo de control no reconocido en state_cb()")
            return

        p_actuation = self.kp * (self.error - self.error_1)
        msg = Float64()
        msg.data = self.u_k_1 + p_actuation

        if self.tipo_control == "controlador_lineal":
            self.actuation_pub_lineal.publish(msg)
        elif self.tipo_control == "controlador_angular":
            self.actuation_pub_angular.publish(msg)

        self.u_k_1 = msg.data
        self.error_2 = self.error_1
        self.error_1 = self.error

    def reset(self):
        self.u_k = 0.0
        self.u_k_1 = 0.0
        self.error = 0.0
        self.error_1 = 0.0
        self.error_2 = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = Controlador_P( 0.5 )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()