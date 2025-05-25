#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String, Float64MultiArray
from threading import Event

class Robot(Node):

    def __init__(self):
        super().__init__('robot')
        # Suscripción a Pose_Loader
        self.suscripcion_array = self.create_subscription(PoseArray,'goal_list',self.accion_mover,10)
        
        # Publicamos en el tópico al controlador
        self.publicar_setpoint = self.create_publisher( Float64MultiArray, 'setpoint', 1 )
        
        # Suscribimos para obtener la respuesta del controlador. 
        # AGREGAR: Tiene que ser en otro hilo, no en el hilo main
        self.suscripcion_controlador = self.create_subscription(String, '/check_control', self.callback_esperar, 10)

        self.evento_esperar = Event()
     
        # Variable para guardar la pose anterior
        self.get_logger().info('Nodo robot iniciado.')
    
    def accion_mover(self, msg):
        """
        Input: msg del topico goal_list recibe una lista de posiciones
        Output: llamar al metodo para actualizar la posicion
        """
        #Recibe la lista de poses objetivo
        self.get_logger().info(f'Recibidas {len(msg.poses)} poses.')
        for i, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            q = pose.orientation
            _, _, theta = euler_from_quaternion((q.x, q.y, q.z, q.w))
            goal = [x, y, theta]
            # self.get_logger().info(f"Moviendo al objetivo {i}: {goal}")
            self.mover_robot_a_destino(goal)


    def mover_robot_a_destino(self, goal):
        """
        Input: posicion destino, odometria (enviada por el topico)
        Output: envia la lista de posiciones
        """

        #! Esto esta ultra-hardcodeado, tener una función que lo cambie
        if goal == [1.0, 1.0, 1.5708]:
            # Desde el inicio queremos ir hasta el extremo opuesto
            lista_desplazamientos = [(1,0),
                                    (0, 1.5708), 
                                    (1,0)
                                    ]
        else:
            # Signifca que queremos volver al inicio
            lista_desplazamientos = [(0, 1.5708),    
                                    (1,0),
                                    (0, 1.5708),    
                                    (1,0),
                                    (0, 1.5708)
                                    ]
        
        # self.get_logger().info(f"Velocidad enviada con el tiempo {msg_to_send}")
        self.aplicar_velocidad(lista_desplazamientos)

    def aplicar_velocidad(self, lista_desplazamientos):
        """
        Recibimos los desplazamientos
        Enviamos cada desplazamiento al controlador 
        Esperamos que el controlador nos indique que terminó de controlar
        """
        for desplazamientos in lista_desplazamientos:
            self.evento_esperar.clear()
            msg = Float64MultiArray()
            msg.data = desplazamientos
            # Publicamos en el tópico la velocidad
            self.publicar_setpoint.publish(msg)

            self.get_logger().info('Esperando ACK...')
            self.evento_esperar.wait()  # Bloquea hasta recibir ack


    def callback_esperar(self, msg):
        # Callback que permite liberar el evento
        self.get_logger().info(f'Msg recibido: {msg.data}')
        self.evento_esperar.set()




def main(args=None):
    rclpy.init(args=args)
    node = Robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
