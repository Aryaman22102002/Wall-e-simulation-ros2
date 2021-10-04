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

    urdf_prefix = get_package_share_directory("my_bot")
    urdf_file = os.path.join(urdf_prefix, "urdf", "m2wr.xacro")

    world_prefix = get_package_share_directory("my_bot")
    world_file = os.path.join(world_prefix, "worlds", "white.world")

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
                    "m2wr",
                    "-x",
                    "-1",
                    "-y",
                    "0",
                    "-z",
                    ".41",
                    "-b",
                    "-file",
                    urdf_file,
                ],
            ),
            Node(
                package="robot_state_publisher",
                node_executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
        ]
    )