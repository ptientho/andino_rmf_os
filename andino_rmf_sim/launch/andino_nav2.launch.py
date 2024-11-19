'''
This is the main launch file for

1. Common launch - RMF resources | Fleet manager | Fleet adapter
2. Simulation launch - Andino fleet | world | custom controllers or nav2 controllers
'''
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # Declare launch configuration
    nav2_enabled = LaunchConfiguration('nav2')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='false', description='If yes, use nav2 controllers.')
    
    # Include Spawn multiple robot launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_fleet'), 'launch'),
            '/spawn_multiple_robot.launch.py'
        ]),
        condition=UnlessCondition(nav2_enabled),
    )
    
    # Include Spawn multiple robot with nav2 launch
    sim_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_fleet'), 'launch'),
            '/spawn_multiple_robot_nav2.launch.py'
        ]),
        condition=IfCondition(nav2_enabled),
    )

    fleet_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('andino_fleet'), 'launch'),
            '/andino_fleet_manager.launch.py'
        ]),
        launch_arguments=[('nav2', nav2_enabled)]
    )

    
    ld = LaunchDescription()
    ld.add_action(nav2_arg)
    ld.add_action(sim_launch)
    ld.add_action(sim_nav2_launch)
    ld.add_action(fleet_manager)
    
    return ld
