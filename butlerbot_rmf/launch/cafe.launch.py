import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare('butlerbot_rmf').find('butlerbot_rmf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    #viz_config_file = LaunchConfiguration('viz_config_file') #TODO


    declare_arguments = [

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use gazebo clock'
        ),
    ]

    # Common Launch
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'common.launch.py'))
    )

    # Fleet Adaptor

    

    return LaunchDescription(
        declare_arguments + [
            common_launch,

        ]
    )