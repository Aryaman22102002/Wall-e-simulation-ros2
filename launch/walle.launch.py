import launch
from launch import actions
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='my_bot').find('my_bot')
    world_path=os.path.join(pkg_share, 'worlds/white.world')

    return launch.LaunchDescription([
       launch_ros.actions.Node(package='my_bot', executable = 'world', output = 'screen'),
        
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

    ])


    
    
    
    
    

