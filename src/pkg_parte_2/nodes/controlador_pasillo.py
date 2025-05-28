#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
# Este nodo recibe de dos topicos
# EL primer tópico es el setpoint y el segundo es el dl_dr

# Del primero se obtiene cual es el deseo  (es  siempre cero, 
# pero el controladro debe estar separado del nodo que setea el deseo)
# y del segundo obtiene la distancia a las paredes.

# Siempre que exista un setpoint vamos a revisar la distnacia y actuará el PID
# la respuesta sería en términos de velocidad angular y envía esto en el tópico

#El topico de publicacion de la velocidad angular sera /control_effort

class HallwayController(Node):
    def __init__(self):
        super().__init__('controlador_pasillo')
        self.get_logger().info('Controlador de pasillo iniciado')
        self.setpoint = None
        self.distance = None
        self.control_effort = Float64()
        self.angular_velocity = None
        # Tiempo
        self.last_time = self.get_clock().now()
        self.current_time = None
        self.dt = None

        #Variables del PID
        self.kp = 0.5
        self.ki = 0.1  # Ganancia integral
        self.kd = 0.01

        self.previous_error = 0.0
        self.current_error = 0.0

        self.integral_actuator = 0.0
        self.derivative_actuator = 0.0
        self.proportional_actuator = 0.0

        # Suscribirse al setpoint
        self.create_subscription(Float64, '/setpoint', self.setpoint_callback, 1)
        # Suscribirse a las distancias
        self.create_subscription(Float64, '/dl_dr', self.distance_callback, 1)
        # Publicar el esfuerzo de control
        self.control_effort_pub = self.create_publisher(Float64, '/control_effort', 1)

    def publish_control_effort(self):
        self.control_effort.data = self.angular_velocity
        self.control_effort_pub.publish(self.control_effort)
        self.get_logger().info(f'Publicando esfuerzo de control: {self.control_effort.data}')

    def setpoint_callback(self, msg):
        self.setpoint = msg.data
        self.get_logger().info(f'Setpoint recibido: {self.setpoint}')

    def distance_callback(self, msg):
        self.distance = -2 #msg.data
        self.get_logger().info(f'Distancia recibida: {self.distance}')
        # Se llama al lazo de control
        self.control_loop()

    def control_loop(self):
        if self.setpoint is None or self.distance is None:
            self.get_logger().warn('Setpoint o distancia no recibidos, no se puede controlar.')
            return
        state = self.distance #Ya que la distancia viene calculada desde el nodo de distancia

        #tiempo
        self.current_time = self.get_clock().now()
        self.dt = (self.current_time - self.last_time).nanoseconds * 1e-9 # pasar a segundos
        self.last_time = self.current_time # actualizar el ultimo tiempo

        # Calcular el error
        self.current_error = self.setpoint - state

        #PID
        self.proportional_actuator = self.kp * self.current_error

        self.integral_actuator += self.ki*(self.current_error * self.dt)

        self.derivative_actuator = self.kd * ((self.current_error - self.previous_error) / self.dt)

        #Guardar el error actual para el siguiente ciclo
        self.previous_error = self.current_error

        self.angular_velocity = self.proportional_actuator + self.integral_actuator + self.derivative_actuator
        self.get_logger().info(f'Control Effort (Angular Velocity): {self.angular_velocity}')
        # Llama al publicador de control_effort
        self.publish_control_effort()





def main(args=None):
    rclpy.init(args=args)
    node = HallwayController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
