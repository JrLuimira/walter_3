import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import xacro

def generate_launch_description():
    # Nombre del paquete y archivo Xacro
    package_description = "walter_description"
    xacro_file = os.path.join(
        get_package_share_directory(package_description),
        'urdf',
        'robot.xacro'
    )

    # Procesar el archivo Xacro para obtener la descripción del robot
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_config}],
        output="screen"
    )

    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory(package_description),
                'config',
                'default.rviz'
            )
        ],
        parameters=[{'use_sim_time': True}]
    )

   
    
    
    

    # Descripción del lanzamiento
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])
