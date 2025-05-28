#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from parametros import Kp, Ki, Kd
from geometry_msgs.msg import Point

class ControladorAngular( Node ):

    def __init__( self):

        super().__init__( 'controlador_angular' )
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.setpoint = None
        self.state = None
        # Definimos las variables para utilizar el controlador PID en tiempo discreto
        self.actuation_last = 0 # u(t_{k-1})
        self.actuacion = 0 # U(t_{k})
        self.t = 1 # Diferencial de tiempo
        self.error = 0 # e(t_{k})
        self.error_last = 0 # #e(t_{k-1})
        self.error_last_last = 0 # #e(t_{k-2})

        self.publicador_actuador = self.create_publisher( Float64, 'respuesta_control', 1 )
        self.suscripcion_setpoint = self.create_subscription( Point, 'setpoint_state', self.recibir_setpoint_state, 1 )
        
    def recibir_setpoint_state(self, msg):
        # Es la callback que recibe la información proveniente de follow_the_carrot
        #self.get_logger().info(f"CTRL: Llegó: {msg.x, msg.y}")
        
        self.setpoint = msg.x
        self.state = msg.y
        self.calcular_accion()

    def calcular_accion(self):
        
        self.error = self.setpoint - self.state

        # Proportional
        p_actuation = self.kp*(self.error -self.error_last)

        # Integrative
        i_actuation = self.ki*(self.t *self.error)

        # Derivative
        d_actuation = self.kd*((self.error/self.t) -(2*self.error_last/self.t) + (self.error_last_last/self.t))

        # Actuation
        self.actuation = self.actuation_last + p_actuation + i_actuation + d_actuation
        #self.get_logger().info(f"La senal de actuacion es {self.actuation}")

        # Actualizamos las futuras variables
        self.error_last_last = self.error_last
        self.error_last = self.error
        self.actuation_last = self.actuacion

        #self.get_logger().info(f"CTRL: Envía{self.actuation}")

        # Message sending
        msg = Float64()
        msg.data = self.actuation
        self.publicador_actuador.publish( msg )

def main(args=None):
    rclpy.init(args=args)
    node = ControladorAngular()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()