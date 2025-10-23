# rqt_dashboard.launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    pkg_trxbewi_description = get_package_share_directory('trxbewi_description')
    rqt_config_path = os.path.join(pkg_trxbewi_description, "rqt/trxbewi_dashboard.perspective")

    # Exemples d’ENV utiles
    env_scale = SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '1')

    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_dashboard',
        output='screen',
        arguments=[
            '--perspective-file',  rqt_config_path,
            # Optionnel: repartir d’une config vierge
            #'--clear-config',
        ],
        # Exemple de remaps si vos topics changent souvent
        remappings=[
            # ('/camera/image_raw', '/camera/image_raw'),  # à adapter si besoin
        ],
    )

    # Petit délai pour laisser le graphe ROS se peupler avant d’ouvrir rqt
    delayed_rqt = TimerAction(period=1.0, actions=[rqt])

    return LaunchDescription([
        env_scale,
        delayed_rqt
    ])
