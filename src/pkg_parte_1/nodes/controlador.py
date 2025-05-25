#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry
import numpy as np

class PIDController( Node ):

	def __init__( self, kp, ki = 0, kd = 0 ):
		super().__init__( 'Controlador' )
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.setpoint_vel_lineal = None
		self.setpoint_vel_angular = None
		self.state = None
		
		# Definimos las variables para utilizar el controlador PID en tiempo discreto
		self.actuation_last = 0 # u(t_{k-1})
		self.actuacion = 0 # U(t_{k})
		self.t = 1 # Diferencial de tiempo
		self.error = 0 # e(t_{k})
		self.error_last = 0 # #e(t_{k-1})
		self.error_last_last = 0 # #e(t_{k-2})
		
		# Nos suscribimos para leer el setpoint y decidir si es lineal y angular el controlado
		self.dist_set_point_sub = self.create_subscription( Float64MultiArray, 'setpoint', self.setpoint_callback, 1 )
		
		# Nos suscribimos a la odometria 
		self.odom_sub = self.create_subscription( Odometry, '/odom', self.odometry_callback, 10 )
		
		# No implementado aún
		self.actuation_pub = self.create_publisher( Float64, 'control_effort', 1 )
	
	def setpoint_callback( self, msg ):
		self.get_logger().info(f'[CTRL] Nuevo setpoint recibido: {msg}')
		self.reset()
		# Desglosamos el mensaje
		self.setpoint_pose_lineal, self.pose_angular = msg.data[0], msg.data[1]

	def odometry_callback(self, odom):
		x = odom.pose.pose.position.x
		y = odom.pose.pose.position.y
        # AGREGAR el de la orientación angular
		if self.setpoint_vel_lineal != None:
			# Debemos ejecutar el PID de velocidad lineal
			# Lo que debe hacer es guardar las coordenadas de la odometría de (x, y) inicial,
			# Sabemos que queremos movernos en terminos absolutos 1 metro
			# Enviamos el mensaje twist
			# Revisamos si la odometria actual avanzó 1 metro (distancia euclideana)

			# Debemos tener otro método para el PID de velocidad angular
			pass

	def state_cb( self, msg ):
		# Este método no esta implementado
		self.state = msg.data
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

		# Message sending
		msg = Float64()
		msg.data = self.actuation
		self.actuation_pub.publish( msg )

	def reset( self ):
		self.setpoint_vel_lineal = None
		self.setpoint_vel_angular = None
		self.state = None


def main():
	rclpy.init()
	# Constantes definidas en el enunciado de la minitarea
	p_ctrl = PIDController(0.5, 0.1,0.01)
	rclpy.spin( p_ctrl )

if __name__ == '__main__':
	main()



