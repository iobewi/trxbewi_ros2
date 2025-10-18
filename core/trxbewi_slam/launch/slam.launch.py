from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # --- Args
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("trxbewi_slam"),
            "config",
            "config.yaml"  # dispo dans trxbewi_slam
        ),
        description="Chemin du fichier YAML de configuration trxbewi_slam"
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="Topic LaserScan d'entrée"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Utiliser /clock (Gazebo/Sim)"
    )

    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # --- SLAM 2D
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",   # alternatif: online_async
        name="slam_toolbox",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("scan", scan_topic)]      # remap propre (sans slash initial)
    )

    # --- Map server (optionnel en mode mapping live, utile pour services/sauvegarde)
    map_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": ""}, {"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        params_file_arg,
        scan_topic_arg,
        use_sim_time_arg,
        slam_node,
        map_node,
    ])
