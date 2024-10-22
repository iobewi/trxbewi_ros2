import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_trxbewi_control = get_package_share_directory("trxbewi_control")
    pkg_trxbewi_description = get_package_share_directory("trxbewi_description")
    pkg_trxbewi_teleop = get_package_share_directory("trxbewi_teleop")

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_trxbewi_description, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={ 'use_simulation': 'false', 'use_controllers': 'true'  }.items(),
    )
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_trxbewi_control, "launch", "control_manager.launch.py")
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_trxbewi_description, "launch", "rviz.launch.py")
        )
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_trxbewi_teleop, "launch", "teleop.launch.py")
        ),
        launch_arguments={"gui": "false"}.items(),
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            control,
            rviz,
            teleop,
        ]
    )