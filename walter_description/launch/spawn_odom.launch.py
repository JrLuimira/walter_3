import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)


def generate_launch_description():
    #walter_description_dir = FindPackageShare(package="walter_description").find("walter_description")

    # Parameters
    x_arg = DeclareLaunchArgument("x", default_value="0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.01", description="Z position")
    package_description = "walter_description"
    # Define robot_description using xacro
    walter_description_dir = FindPackageShare(package="walter_description").find(
        "walter_description"
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([walter_description_dir, "urdf", "robot.xacro"]),
        ]
    )  

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }
    
    #print(robot_description_content.perform(context=context))
    # Spawn the model in Gazebo
    spawn_model_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="walter_spawn",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        arguments=[
            "-entity",
            "walter",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
        ],
    )

    # Load controllers
    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        remappings=[
            ("/diff_drive_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    # Launch description
    return LaunchDescription([
        x_arg,  # Argumento para 'x'
        y_arg,  # Argumento para 'y'
        z_arg,  # Argumento para 'z'
        spawn_model_node,
        joint_state_broadcaster_spawner_node,
        robot_controller_spawner,
])

