"""A minimal 2D robot simulator used by the demo scripts.

It owns the *true* pose (which the localizer is not allowed to see) and produces:
  * noisy **odometry** poses (accumulated drift), and
  * **laser scans** via ray casting against the map.

Just enough to exercise the from-scratch filter without ROS or Gazebo.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amcl.utils import normalize_angle  # noqa: E402


class RobotSimulator:
    def __init__(self, grid, true_pose, num_beams=180, fov=2 * np.pi,
                 max_range=8.0, range_sigma=0.02,
                 odom_trans_sigma=0.02, odom_rot_sigma=0.01):
        self.grid = grid
        self.true_pose = np.asarray(true_pose, dtype=float)
        self.odom_pose = np.array(true_pose, dtype=float)  # starts aligned
        self.angles = np.linspace(-fov / 2, fov / 2, num_beams, endpoint=False)
        self.max_range = max_range
        self.range_sigma = range_sigma
        self.odom_trans_sigma = odom_trans_sigma
        self.odom_rot_sigma = odom_rot_sigma

    def move(self, d_trans, d_rot):
        """Drive ``d_trans`` meters forward then turn ``d_rot`` radians.

        Updates both the true pose and a drifting odometry estimate.
        """
        th = self.true_pose[2]
        self.true_pose[0] += d_trans * np.cos(th)
        self.true_pose[1] += d_trans * np.sin(th)
        self.true_pose[2] = normalize_angle(th + d_rot)

        # Odometry sees a noisy version of the same motion (drift accumulates).
        noisy_trans = d_trans + np.random.normal(0, self.odom_trans_sigma)
        noisy_rot = d_rot + np.random.normal(0, self.odom_rot_sigma)
        oth = self.odom_pose[2]
        self.odom_pose[0] += noisy_trans * np.cos(oth)
        self.odom_pose[1] += noisy_trans * np.sin(oth)
        self.odom_pose[2] = normalize_angle(oth + noisy_rot)

    def scan(self):
        """Return (ranges, angles) for a laser scan from the *true* pose."""
        x, y, th = self.true_pose
        ranges = np.array([
            self.grid.ray_cast(x, y, th + a, self.max_range)
            for a in self.angles
        ])
        ranges += np.random.normal(0, self.range_sigma, size=ranges.shape)
        ranges = np.clip(ranges, 0, self.max_range)
        return ranges, self.angles.copy()
