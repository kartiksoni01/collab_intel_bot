import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = os.path.join(get_package_share_directory('bal_robot_bringup'), 'launch')
    navigation_dir = os.path.join(get_package_share_directory('bal_robot_navigation'), 'launch')
    rviz_launch_dir = os.path.join(get_package_share_directory('bal_robot_description'), 'launch')
    gazebo_launch_dir = os.path.join(get_package_share_directory('bal_robot_gazebo'), 'launch')
    cartographer_launch_dir = os.path.join(get_package_share_directory('bal_robot_slam'), 'launch')
    map_directory = os.path.join(get_package_share_directory('bal_robot_navigation'), 'maps', 'room2.yaml')
    rviz_config_path = os.path.join(get_package_share_directory('bal_robot_description'), 'rviz/navigation.rviz')
    ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration = LaunchConfiguration('exploration')


    rviz_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
        condition=IfCondition(use_sim_time),
        launch_arguments={'use_sim_time': use_sim_time,
                          "rvizconfig": rviz_config_path}.items())

    state_publisher_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
        condition=IfCondition(use_sim_time),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')),
        launch_arguments={'exploration': exploration,
                          'map_file': map_file,
                          'use_sim_time': use_sim_time}.items())

    cartographer_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_dir, 'cartographer.launch.py')),
        launch_arguments={'exploration': exploration,
                          'use_sim_time': use_sim_time}.items())

    ydlidar_launch_cmd=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ydlidar_launch_dir, 'ydlidar_launch.py')),
                condition=IfCondition(PythonExpression(['not ', use_sim_time])),
                launch_arguments={'use_sim_time':use_sim_time}.items())   
    
    mavros_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'mavros.launch.py')),
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                             description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='exploration', default_value='True',
                                             description='Flag to enable exploration'),
        DeclareLaunchArgument(name='map_file', default_value=map_directory,
                                              description='Map to be used'),

        rviz_launch_cmd,
        ydlidar_launch_cmd,
        state_publisher_launch_cmd,
        gazebo_launch_cmd,
        navigation_launch_cmd,
        cartographer_launch_cmd,
        mavros_launch_cmd

    ])

