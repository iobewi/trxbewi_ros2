# rqt_standalone.launch.py
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '1'),
        SetEnvironmentVariable('ROS_IMAGE_TRANSPORT', 'compressed'),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_standalone',
            output='screen',
            arguments=[
                '--standalone', 'rqt_image_view',
                '--standalone', 'rqt_plot',
                '--standalone', 'rqt_topic',
                # Ajoute d’autres plugins si besoin
            ],
        ),
    ])
