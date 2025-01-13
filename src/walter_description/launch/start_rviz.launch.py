import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nombre del paquete
    package_description = "walter_description"

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
        rviz_node,
    ])
