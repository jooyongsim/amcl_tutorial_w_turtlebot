"""Drive the robot through a list of waypoints using nav2_simple_commander.

This shows the "use the packages" payoff: once AMCL is localizing and Nav2 is
running, navigation is a few lines — you set an initial pose and hand goals to
the BasicNavigator; the planner/controller (and AMCL underneath) do the rest.

    ros2 run amcl_nav_pkg waypoint_navigator
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw=0.0):
    """Build a PoseStamped in the map frame from (x, y, yaw)."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 1. Tell AMCL where we start (equivalent to RViz "2D Pose Estimate").
    initial = make_pose(navigator, 2.0, 2.0, 0.0)
    navigator.setInitialPose(initial)

    # 2. Wait until Nav2 (and AMCL) are up and active.
    navigator.waitUntilNav2Active()

    # 3. A simple patrol around the tutorial map.
    waypoints = [
        make_pose(navigator, 7.0, 2.0, 0.0),
        make_pose(navigator, 7.0, 8.0, math.pi / 2),
        make_pose(navigator, 2.0, 8.0, math.pi),
        make_pose(navigator, 2.0, 2.0, -math.pi / 2),
    ]

    for i, goal in enumerate(waypoints, 1):
        navigator.get_logger().info(
            f"Going to waypoint {i}/{len(waypoints)}: "
            f"({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
        navigator.goToPose(goal)

        # Poll feedback while the task runs.
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and feedback.distance_remaining is not None:
                navigator.get_logger().info(
                    f"  {feedback.distance_remaining:.2f} m remaining", throttle_duration_sec=1.0)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f"  reached waypoint {i}")
        else:
            navigator.get_logger().warn(f"  failed at waypoint {i}: {result}")
            break

    navigator.get_logger().info("Patrol complete.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
