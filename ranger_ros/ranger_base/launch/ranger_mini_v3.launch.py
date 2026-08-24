import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true')

    port_name_arg = DeclareLaunchArgument('port_name', default_value='can0',
                                         description='CAN bus name, e.g. can0')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')

    publish_odom_tf_arg = DeclareLaunchArgument('publish_odom_tf', default_value='false',
                                                 description='Publish odom to base TF')
    update_rate_arg = DeclareLaunchArgument('update_rate', default_value='50',
                                             description='Driver update rate')
    robot_model_arg = DeclareLaunchArgument('robot_model', default_value='ranger_mini_v3',
                                           description='robot motion model')
    cmd_vel_timeout_arg = DeclareLaunchArgument('cmd_vel_timeout', default_value='0.25')
    stop_velocity_threshold_arg = DeclareLaunchArgument(
        'stop_velocity_threshold', default_value='0.01')
    stop_wheel_speed_threshold_arg = DeclareLaunchArgument(
        'stop_wheel_speed_threshold', default_value='0.01')
    steering_center_tolerance_arg = DeclareLaunchArgument(
        'steering_center_tolerance', default_value='0.01')
    stop_wait_timeout_arg = DeclareLaunchArgument('stop_wait_timeout', default_value='2.0')
    mode_switch_timeout_arg = DeclareLaunchArgument('mode_switch_timeout', default_value='2.0')
    centering_timeout_arg = DeclareLaunchArgument('centering_timeout', default_value='3.0')
    stop_stable_frames_arg = DeclareLaunchArgument('stop_stable_frames', default_value='3')
    center_stable_frames_arg = DeclareLaunchArgument('center_stable_frames', default_value='3')
    ranger_base_node = launch_ros.actions.Node(
        package='ranger_base',
        executable='ranger_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'port_name': launch.substitutions.LaunchConfiguration('port_name'),                
                'odom_frame': launch.substitutions.LaunchConfiguration('odom_frame'),
                'base_frame': launch.substitutions.LaunchConfiguration('base_frame'),
                'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name'),
                'publish_odom_tf': launch.substitutions.LaunchConfiguration('publish_odom_tf'),
                'update_rate': launch.substitutions.LaunchConfiguration('update_rate'),
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'cmd_vel_timeout': launch.substitutions.LaunchConfiguration('cmd_vel_timeout'),
                'stop_velocity_threshold': launch.substitutions.LaunchConfiguration('stop_velocity_threshold'),
                'stop_wheel_speed_threshold': launch.substitutions.LaunchConfiguration('stop_wheel_speed_threshold'),
                'steering_center_tolerance': launch.substitutions.LaunchConfiguration('steering_center_tolerance'),
                'stop_wait_timeout': launch.substitutions.LaunchConfiguration('stop_wait_timeout'),
                'mode_switch_timeout': launch.substitutions.LaunchConfiguration('mode_switch_timeout'),
                'centering_timeout': launch.substitutions.LaunchConfiguration('centering_timeout'),
                'stop_stable_frames': launch.substitutions.LaunchConfiguration('stop_stable_frames'),
                'center_stable_frames': launch.substitutions.LaunchConfiguration('center_stable_frames'),
                }])

    return LaunchDescription([
        use_sim_time_arg,
        port_name_arg,        
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        publish_odom_tf_arg,
        update_rate_arg,
        robot_model_arg,
        cmd_vel_timeout_arg,
        stop_velocity_threshold_arg,
        stop_wheel_speed_threshold_arg,
        steering_center_tolerance_arg,
        stop_wait_timeout_arg,
        mode_switch_timeout_arg,
        centering_timeout_arg,
        stop_stable_frames_arg,
        center_stable_frames_arg,
        ranger_base_node
    ])
