#
#  qrcode_reader.launch.py
#  qrcode_reader
#
#  Python launch file for ROS 2
#

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qrcode_reader',
            executable='qrcode_reader',
            output='screen',
            parameters=[ {
                'camera': '/stereo/left/image',
                'debug_img': True,
            }]
        ),
    ])