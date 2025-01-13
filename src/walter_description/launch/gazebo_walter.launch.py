import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
import xacro


def generate_launch_description():
    # Nombre del paquete y archivo Xacro
    package_name = "walter_description"
    xacro_file = os.path.join(
        get_package_share_directory(package_name), "urdf", "robot.xacro"
    )

    controller_path = os.path.join(
        get_package_share_directory(package_name), "config", "controllers.yaml"
    )


    # Procesar el archivo Xacro para obtener la descripción del robot
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Include the spawn.launch.py
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    xacro_file,
                    "launch",
                    "spawn_odom.launch.py",
                ]
            )
        ),
        launch_arguments={"y": "2"}.items(),
    )

    # Nodo robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True, "robot_description": robot_description_config}
        ],
    )
    # tf broadcaster node
    groundtruth_broadcaster_node = Node(
        package="walter_description",
        executable="groundtruth_broadcaster",
        name="groundtruth_broadcaster_node",
        output="screen",
        parameters=[{"init_pose_y": 2}],
    )

    # Incluir el launch de Gazebo desde el paquete `gazebo_ros`
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
         launch_arguments={
            "verbose": "true",
         }.items(),
    )


    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[{"use_sim_time": True}, controller_path],
    )

    # Descripción del lanzamiento
    return LaunchDescription(
        [
            gazebo_launch,
            robot_state_publisher,
            controller_manager,
            joint_state_publisher_node,
            spawn_launch,
            groundtruth_broadcaster_node,
        ]
    )
