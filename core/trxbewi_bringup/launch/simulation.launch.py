from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_trxbewi_simulation = get_package_share_directory("trxbewi_simulation")
    pkg_trxbewi_description = get_package_share_directory('trxbewi_description')
    pkg_trxbewi_teleop = get_package_share_directory('trxbewi_teleop')



    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_trxbewi_simulation, 'launch', 'gz.launch.py'))
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_trxbewi_description, 'launch', 'rviz.launch.py'))
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_trxbewi_teleop, 'launch', 'teleop.launch.py')),
        launch_arguments={ 'gui': 'true' }.items(),
    )

    return LaunchDescription([
        simulation,
        rviz,
        teleop,
    ])