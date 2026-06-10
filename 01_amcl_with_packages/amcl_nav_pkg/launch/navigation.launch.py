"""Launch the full navigation stack: AMCL localization + Nav2.

This composes our localization launch with the standard Nav2 bringup (planner,
controller, behavior tree, costmaps). After launch:

    ros2 launch amcl_nav_pkg navigation.launch.py

then in RViz: "2D Pose Estimate" to localize, "Nav2 Goal" to send a goal.
You can also drive waypoints programmatically with:

    ros2 run amcl_nav_pkg waypoint_navigator
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("amcl_nav_pkg")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    default_map = os.path.join(pkg, "maps", "tutorial_map.yaml")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_args = [
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    # Reuse Nav2's well-tested bringup, but point it at OUR map + params (which
    # contain the AMCL block from config/amcl.yaml, merged into nav2_params.yaml).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(declare_args + [nav2])
