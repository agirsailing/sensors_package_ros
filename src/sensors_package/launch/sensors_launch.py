from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors_package',
            executable='talker_ultrasonic',
            name='ultrasonic_left',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'topic': 'ctrl_raw_left',
                'device_id': 'ultrasonic_left',
            }]
        ),
        Node(
            package='sensors_package',
            executable='talker_ultrasonic',
            name='ultrasonic_right',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'topic': 'ctrl_raw_right',
                'device_id': 'ultrasonic_right',
            }]
        ),
        Node(
            package='sensors_package',
            executable='talker_filter_ultrasonic',
            name='ultrasonic_filter_node',
        ),
    ])