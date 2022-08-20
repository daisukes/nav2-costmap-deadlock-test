import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.logging import launch_config

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('deadlock_test')

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_dir, 'test', 'test.rviz'),
            description='Full path to the RVIZ config file to use'),
    
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'test', 'test.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_footprint_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
            parameters=[configured_params],
            remappings=remappings
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='log'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['test_node']}
                        ]
        ),
        Node(
            package='deadlock_test',
            executable='test_node',
            name='test_node',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        )
        ])
