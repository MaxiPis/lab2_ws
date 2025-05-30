from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription



def generate_launch_description():
    # Cargamos la ruta para levantar el launch de la simulacion
    simulacion_launch = os.path.join(
                        get_package_share_directory('very_simple_robot_simulator'),
                                                     'launch/run_all.xml')

    # Cargamos nuestros nodos
    publicador_velocidad = Node(package='pkg_parte_3',
                               executable='publicador_velocidad.py',
                               name='publicador_velocidad')
    
    controlador_angular = Node(
           package='pkg_parte_3',
            executable='controlador_angular.py',
            name='controlador_angular'
        )
    
    follow_the_carrot = Node(
            package='pkg_parte_3',
            executable='follow_the_carrot.py',    
            name="follow_the_carrot"
            )


    publicador_ruta = Node(
            package='pkg_parte_3',
            executable='publicador_ruta.py',
            name='publicador_ruta')
    
    levantar_ventana = Node(
        package='pkg_parte_3',
            executable='mostrar_ruta.py',
            name='mostrarr_ruta')
    # Retornamos el archivo de tipo launch
    launch = LaunchDescription([
        IncludeLaunchDescription(XMLLaunchDescriptionSource(simulacion_launch)),
        publicador_velocidad,
        controlador_angular,
        follow_the_carrot,
        publicador_ruta,
        levantar_ventana
        ])
    return launch