"""Launch AMCL localization only: map_server + amcl + lifecycle manager.

This is the minimal "Where am I?" pipeline (slides/06). First bring up a robot /
simulator that publishes /scan and the odom->base_frame TF (see SIMULATION.md
for a full TurtleBot3 + Gazebo walkthrough), then:

    ros2 launch amcl_nav_pkg localization.launch.py map:=<your_map>.yaml use_sim_time:=true

In RViz, click "2D Pose Estimate" to seed AMCL, then drive the robot and watch
the particle cloud converge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("amcl_nav_pkg")
    default_map = os.path.join(pkg, "maps", "tutorial_map.yaml")
    default_amcl = os.path.join(pkg, "config", "amcl.yaml")

    map_yaml = LaunchConfiguration("map")
    amcl_params = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_args = [
        DeclareLaunchArgument("map", default_value=default_map,
                              description="Path to the map .yaml"),
        DeclareLaunchArgument("params_file", default_value=default_amcl,
                              description="Path to the AMCL params .yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use /clock from the simulator"),
    ]

    # The lifecycle nodes AMCL localization needs.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params, {"use_sim_time": use_sim_time}],
    )

    # Lifecycle manager drives map_server + amcl through configure -> activate.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    return LaunchDescription(declare_args + [map_server, amcl, lifecycle_manager])
