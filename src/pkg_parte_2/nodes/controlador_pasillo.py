#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class HallwayController(Node):
    def __init__(self):
        super().__init__('controlador_pasillo')
        self.get_logger().info('Controlador de pasillo iniciado')

        # Estado y referencias
        self.setpoint = None
        self.distance = None

        # Parámetros del PID
        self.kp = 0.3
        self.ki = 0.0001
        self.kd = 0.000001

        self.previous_error = 0.0
        self.integral_actuator = 0.0

        self.angular_velocity = 0.0
        self.last_time = self.get_clock().now()

        # Temporizador del control
        self.timer = self.create_timer(0.1, self.control_loop)

        # Suscripciones
        self.create_subscription(Float64, 'setpoint', self.setpoint_callback, 1)
        self.create_subscription(Float64, 'dl_dr', self.distance_callback, 1)

        # Publicador
        self.control_effort_pub = self.create_publisher(Float64, 'control_effort', 1)

    def setpoint_callback(self, msg):
        try:
            self.setpoint = float(msg.data)
        except ValueError:
            self.get_logger().warn("Setpoint recibido no es un número válido.")

    def distance_callback(self, msg):
        try:
            self.distance = float(msg.data)
            self.get_logger().info(f'Distancia recibida: {self.distance}')
        except ValueError:
            self.get_logger().warn("Distancia recibida no es un número válido.")

    def control_loop(self):
        if self.setpoint is None or self.distance is None:
            self.get_logger().info('Esperando setpoint y distancia...')
            return

        try:
            state = float(self.distance)
            setpoint = float(self.setpoint)
        except ValueError:
            self.get_logger().warn("Valores no válidos para el control.")
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0:
            dt = 1e-6  # previene división por cero

        # Cálculo del error
        current_error = setpoint - state
        if not isinstance(current_error, float) or math.isnan(current_error):
            current_error = self.previous_error

        # PID
        proportional = self.kp * current_error
        self.integral_actuator += self.ki * current_error * dt
        derivative = self.kd * (current_error - self.previous_error) / dt

        self.angular_velocity = proportional + self.integral_actuator + derivative
        self.previous_error = current_error

        # Publicar resultado
        self.publish_control_effort()

    def publish_control_effort(self):
        msg = Float64()
        msg.data = self.angular_velocity
        self.control_effort_pub.publish(msg)
        self.get_logger().info(f'Publicando esfuerzo de control: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = HallwayController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
