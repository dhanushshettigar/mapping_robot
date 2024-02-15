import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    baud_rate = LaunchConfiguration('baud_rate', default='9600')
    subscribe_to = LaunchConfiguration('subscribe_to', default='my_data')
    publish_to = LaunchConfiguration('publish_to', default='my_data')

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port2 = LaunchConfiguration('serial_port2', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected arduino'),

        DeclareLaunchArgument(
            'baud_rate',
            default_value=baud_rate,
            description='Specifying usb port baudrate to connected arduino'),
        
        DeclareLaunchArgument(
            'subscribe_to',
            default_value=subscribe_to,
            description='Specifying topic to subscribe values to write'),

        DeclareLaunchArgument(
            'publish_to',
            default_value=publish_to,
            description='Specifying topic to publish values'),

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port2',
            default_value=serial_port2,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),


        Node(
            package='ros2_serial_arduino',
            executable='serial_writer',
            name='serial_data_writer',
            parameters=[{'serial_port':serial_port,
                         'baud_rate': baud_rate, 
                         'subscribe_to': subscribe_to}],
            output='screen'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port2,
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate}],
            output='screen'),

        Node(
            package='ros2_rplidar_sub',
            executable='scan_listener',
            name='rplidar_viewer',
            output='screen'),       
    ])
