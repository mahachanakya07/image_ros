from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=[{'framerate': 30.0, 'image_width': 640, 'image_height': 480}]
        ),
        Node(
            package='image_ros',
            executable='image_conversion_node',
            name='image_conversion_node',
            parameters=[{
                'input_topic': '/usb_cam/image_raw',
                'output_topic': '/image_converted'
            }]
        )
    ])
