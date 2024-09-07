from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the configuration files
    pluginlists_yaml = get_package_share_directory('ci_robot') + '/config/apm_pluginlists.yaml'
    config_yaml = get_package_share_directory('ci_robot') + '/config/apm_config.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', 
            default_value='/dev/ttyAMA0:57600',
            description='FCU connection URL'),

        DeclareLaunchArgument(
            'gcs_url', 
            default_value='udp-b://:14550@',
            description='GCS connection URL'),

        DeclareLaunchArgument(
            'tgt_system', 
            default_value='1',
            description='Target system ID'),

        DeclareLaunchArgument(
            'tgt_component', 
            default_value='1',
            description='Target component ID'),

        DeclareLaunchArgument(
            'fcu_protocol', 
            default_value='v2.0',
            description='FCU protocol'),

        DeclareLaunchArgument(
            'respawn_mavros', 
            default_value='true',
            description='Whether to respawn MAVROS if it crashes'),

        DeclareLaunchArgument(
            'namespace', 
            default_value='mavros',
            description='Namespace for MAVROS'),

        # Node definition for MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': LaunchConfiguration('tgt_system'),
                'tgt_component': LaunchConfiguration('tgt_component'),
                'fcu_protocol': LaunchConfiguration('fcu_protocol')
            },
            pluginlists_yaml,
            config_yaml],
            remappings=[
                ('/mavros/setpoint_velocity/cmd_vel_unstamped', '/cmd_vel')
            ]
        ),
    ])
