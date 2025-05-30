from setuptools import setup
import os
from glob import glob

package_name = 'pkg_parte_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Recursos ROS 2
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Archivo coordenadas.txt en share del paquete
        ('share/' + package_name, [os.path.join(package_name, 'coordenadas.txt')]),

        # Archivos launch
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='constanza',
    maintainer_email='cvsepulveda1@uc.cl',
    description='Paquete para cargar poses desde coordenadas.txt y publicarlas en un tópico.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_loader = pkg_parte_1.pose_loader:main',
            'robot = pkg_parte_1.robot:main',
            'controlador_P = pkg_parte_1.controlador_P:main',
            'publicador_vel = pkg_parte_1.publicador_vel:main',
        ],
    },
)