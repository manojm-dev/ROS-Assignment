import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare('butlerbot_rmf_gazebo').find('butlerbot_rmf_gazebo')
    pkg_rmf_visualization_schedule = FindPackageShare('rmf_visualization_schedule').find('rmf_visualization_schedule')
    pkg_rmf_visualization = FindPackageShare('rmf_visualization').find('rmf_visualization')

    rviz_config_file = os.path.join(pkg_rmf_visualization, 'config', 'rmf.rviz')
    building_config_file = os.path.join(pkg_share, 'maps', 'cafe.building.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_map = LaunchConfiguration('initial_map')
    bidding_time_window = LaunchConfiguration('bidding_time_window')
    use_unique_hex_string_with_task_id = LaunchConfiguration('use_unique_hex_string_with_task_id')
    server_uri = LaunchConfiguration('server_uri')

    declare_arguments = [

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use Gazebo Clock'
        ),

        DeclareLaunchArgument(
            name='building_config_file',
            default_value=building_config_file,
            description='Building description file required by rmf_building_map_tools'
        ),

        DeclareLaunchArgument(
            name='initial_map',
            default_value='L1',
            description='Initail map name for the visualizer'
        ),

        DeclareLaunchArgument(
            'bidding_time_window',
            default_value='2.0',
            description='Time window in seconds for task bidding process'
        ),
        DeclareLaunchArgument(
            'use_unique_hex_string_with_task_id',
            default_value='true',
            description='Appends a unique hex string to the task ID'
        ),
        DeclareLaunchArgument(
            'server_uri',
            default_value='',
            description='The URI of the API server to transmit state and task information'
        ),
    ]

    # Traffic Schedule
    traffic_schedule = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_schedule',
        name='rmf_traffic_schedule_primary',
        output='both',
        parameters=[{
            'use_sim_time' : use_sim_time
        }]
    )

    # Blockade Moderator
    traffic_blocker = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_blockade',
        name='rmf_traffic_blockade',
        output='both',
        parameters=[{
            'use_sim_time' : use_sim_time
        }]
    )

    # Building Map
    building_map_server = Node(
        package='rmf_building_map_tools',
        executable='building_map_server',
        name='building_map_server',
        output='both',
        arguments=[building_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
            }],
    )

    # Visualizer
    visualizer = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_rmf_visualization, 'visualization.launch.xml')),
        launch_arguments={
            'use_sim_time'      : use_sim_time,
            'map_name'          : initial_map,
            'viz_config_file'   : rviz_config_file
        }.items()
    )

    # Door Supervisor
    door_supervisor = Node(
        package='rmf_fleet_adapter',
        executable='door_supervisor',
        name='door_supervisor',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time
            }],
    )

    # Lift Supervisor
    lift_supervisor = Node(
        package='rmf_fleet_adapter',
        executable='lift_supervisor',
        name='lift_supervisor',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time
            }],
    )

    #Dispatcher Node
    dispatcher_node = Node(
        package='rmf_task_ros2',
        executable='rmf_task_dispatcher',
        name='rmf_task_dispatcher',
        output='screen',
        parameters=[{
            'use_sim_time'                          : use_sim_time,
            'bidding_time_window'                   : bidding_time_window,
            'use_unique_hex_string_with_task_id'    : use_unique_hex_string_with_task_id,
            'server_uri'                            : server_uri
            }],
    )


    return LaunchDescription(
        declare_arguments +[
            traffic_schedule,
            traffic_blocker,
            building_map_server,
            GroupAction(
                actions=[
                    visualizer,
                    door_supervisor,
                    lift_supervisor,
                    dispatcher_node
                ]
            )
        ]
    )