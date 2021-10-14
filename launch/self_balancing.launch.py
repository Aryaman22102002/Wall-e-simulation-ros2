import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths


def generate_launch_description():
    model_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
    }

    sdf_prefix = get_package_share_directory("my_bot")
    sdf_file = os.path.join(sdf_prefix, "urdf", "walle.sdf")

    world_prefix = get_package_share_directory("my_bot")
    world_file = os.path.join(world_prefix, "worlds", "sra.world")
    
    rviz_config_prefix = get_package_share_directory("my_bot")
    rviz_config_path = os.path.join(rviz_config_prefix, 'rviz/urdf_config.rviz')


    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                     world_file,
                ],
                output="screen",
                additional_env=env,
            ),
            Node(
                package="gazebo_ros",
                node_executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "walle",
                    "-x",
                    "-1",
                    "-y",
                    "0",
                    "-z",
                    ".41",
                    "-b",
                    "-file",
                    sdf_file,
                ],
            ),
            Node(
                package="robot_state_publisher",
                node_executable="robot_state_publisher",
                output="screen",
                arguments=[sdf_file],
            ),
            Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output="screen",
            arguments=[rviz_config_path],
    )
        ]
    )
