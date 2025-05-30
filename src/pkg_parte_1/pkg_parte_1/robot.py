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
        # Comunicación
        self.evento_esperar = Event()
        # Suscripción a Pose_Loader
        self.suscripcion_array = self.create_subscription(PoseArray,'goal_list',self.accion_mover,10)
        # Publicar a los nodos de control
        self.publisher_controlP = self.create_publisher(Float64MultiArray, '/setpoint_P', 10)
        self.pose_pub = self.create_subscription(Float64MultiArray, '/pose_actual',self.callback_esperar, 10)
        # Estado anterior
        self.ultima_posicion = [0.0, 0.0, 0.0]
        
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
            self.mover_robot_a_destino(self.ultima_posicion,goal)


    def mover_robot_a_destino(self, actual, goal):
        """
        Input: listas que se recibe son tipo [x, y, theta], que son las posiciones en las que quiere avanzar (x,y)
               y hacia donde debe mirar la orientacion del robot (theta) usando coordenadas_antiguas.txt
        Output: lista de desplazamientos dependiendo de si es de tipo lineal o angular
        """
        posx_actual = actual[0]
        posy_actual = actual[1]
        theta_actual= actual[2]
        posx_final = goal[0]
        posy_final = goal[1]
        theta_final = goal[2]

        dx = posx_final - posx_actual
        dy = posy_final - posy_actual
        dtheta = theta_final - theta_actual

        lista_desplazamientos = []
        # Descomposicion lista desplazamiento lineal
        if dtheta == 0 and (dx != 0 or dy != 0):
            distancia = (dx**2 + dy**2)**0.5
            lista_desplazamientos.append((distancia, 0.0))
        # Descomposicion lista desplazamiento angular
        elif dx == 0 and dy == 0 and dtheta != 0:
            lista_desplazamientos.append((0.0, -dtheta))
        
        self.get_logger().info(f"X {posx_actual,posx_final}")
        self.get_logger().info(f"Y {posy_actual,posy_final}")
        self.get_logger().info(f"THETA {theta_actual,theta_final}")
        self.aplicar_velocidad(lista_desplazamientos)


    def aplicar_velocidad(self, lista_desplazamientos):
        """
        Input: recibe los desplazamientos.
        Output: se envia cada desplazamiento al controlador lineal o angular según corresponda,
                esperamos que el controlador nos indique que terminó de controlar.
        """
        for desplazamiento in lista_desplazamientos:
            self.evento_esperar.clear()
            msg = Float64MultiArray()
            msg.data = desplazamiento

            # Determinar si es lineal o angular
            if desplazamiento[0] != 0.0 and desplazamiento[1] == 0.0:
                self.get_logger().info(f'Publicando desplazamiento lineal: {desplazamiento}')
                self.publisher_controlP.publish(msg)
            elif desplazamiento[0] == 0.0 and desplazamiento[1] != 0.0:
                self.get_logger().info(f'Publicando desplazamiento angular: {desplazamiento}')
                self.publisher_controlP.publish(msg)

            else:
                self.get_logger().warn(f'Desplazamiento no válido: {desplazamiento}')
                continue  # Lo ignora

            #self.get_logger().info('Esperando ACK...')
            self.evento_esperar.wait(timeout=5)


    def callback_esperar(self, msg):
        # Obtener los datos [x, y, theta]
        nueva_pos = msg.data
        self.get_logger().info(f'Msg recibido: {msg.data}')
        # Enviar la nueva posicion
        self.mover_robot_a_destino(self.ultima_posicion, nueva_pos)
        # Actualizar la posición
        self.ultima_posicion = nueva_pos
        # Liberar evento
        self.evento_esperar.set()


def main(args=None):
    rclpy.init(args=args)
    node = Robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
