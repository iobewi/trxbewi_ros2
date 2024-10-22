from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_trxbewi_simulation = get_package_share_directory("trxbewi_simulation")
    pkg_trxbewi_description = get_package_share_directory('trxbewi_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_trxbewi_control = get_package_share_directory('trxbewi_control')

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_trxbewi_description, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={ 'use_simulation': 'true', 'use_controllers': 'true'  }.items(),
    )
    
    control_spawner= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_trxbewi_control, 'launch', 'control_spawner.launch.py'))
    )

    # Gz launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={ 'gz_args': ' -r ' + pkg_trxbewi_simulation + '/worlds/home.world' }.items(),
    )

    gz_sim_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'robot',
                   '-allow_renaming', 'true',
                   '-z', '0.4'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        gz_sim_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_sim_spawn_entity,
                on_exit=[control_spawner],
            )
        ),
        bridge,
    ])