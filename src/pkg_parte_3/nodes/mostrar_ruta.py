#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import pygame
import sys
from parametros import RUTA_ARCHIVO
import os

class MostrarRuta(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)

        self.trajectory = []
        
        pygame.init()
        self.size = (800, 600)
        self.screen = pygame.display.set_mode(self.size)
        pygame.display.set_caption("Odom Plotter")
        self.clock = pygame.time.Clock()
        self.static_path = self.load_static_path()

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        screen_x = int(x * 20) + self.size[0] // 2
        screen_y = int(-y * 20) + self.size[1] // 2
        self.trajectory.append((screen_x, screen_y))

    def load_static_path(self):
        dir_route = os.path.dirname(os.path.abspath(__file__)) #Consigue la ruta del directorio actual
        
        coordenadas_file_path = os.path.join(dir_route, RUTA_ARCHIVO)
        
        #coordenadas_file_path = "mapas/path_sin.txt"
        coordenadas_file_path = f"mapas/{RUTA_ARCHIVO}"
        coords = []

        dir_route = os.path.dirname(os.path.abspath(__file__)) #Consigue la ruta del directorio actual
        coordenadas_file_path = os.path.join(dir_route, RUTA_ARCHIVO)

        with open(coordenadas_file_path, 'r') as archivo:
            for linea in archivo:
                partes = linea.strip().split()
                x , y = partes[0].split(",")
                x, y = float(x), float(y)
                coords.append((x,y))

        screen_coords = [
            (int(x * 20) + self.size[0] // 2, int(-y * 20) + self.size[1] // 2)
            for x, y in coords
        ]
        return screen_coords

    def draw_static_path(self):
        if len(self.static_path) > 1:
            pygame.draw.lines(self.screen, (255, 0, 0), False, self.static_path, 2)

    def run(self):
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    rclpy.shutdown()
                    sys.exit()

            self.screen.fill((255, 255, 255))
            self.draw_static_path()
            if len(self.trajectory) > 1:
                pygame.draw.lines(self.screen, (0, 0, 255), False, self.trajectory, 2)
            pygame.display.flip()
            self.clock.tick(30)
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    node = MostrarRuta()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
