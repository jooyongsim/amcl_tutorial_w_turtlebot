"""Probabilistic odometry motion model (Thrun et al., *Probabilistic Robotics*).

We decompose the relative motion encoded by two consecutive odometry poses into
``rot1 -> trans -> rot2`` and sample noisy versions for each particle. This is
the *sampling* form (we draw new poses), which is exactly what MCL's prediction
step needs.
"""

import numpy as np

from .utils import normalize_angle, sample_normal


class OdometryMotionModel:
    def __init__(self, alpha=(0.1, 0.1, 0.1, 0.1)):
        """``alpha`` = (a1, a2, a3, a4) noise parameters.

        a1: rotation noise from rotation       a2: rotation noise from translation
        a3: translation noise from translation a4: translation noise from rotation
        """
        self.a1, self.a2, self.a3, self.a4 = alpha

    def sample(self, particles, odom_prev, odom_curr):
        """Advance every particle by the (noisy) motion ``odom_prev -> odom_curr``.

        Parameters
        ----------
        particles : (N, 3) array of current particle poses (x, y, theta).
        odom_prev, odom_curr : (3,) odometry poses (x, y, theta) in the odom frame.

        Returns a new (N, 3) array of predicted poses.
        """
        particles = np.asarray(particles, dtype=float)
        n = len(particles)

        dx = odom_curr[0] - odom_prev[0]
        dy = odom_curr[1] - odom_prev[1]
        trans = np.hypot(dx, dy)

        # Guard against the rot1 being ill-defined for (near) pure rotation.
        if trans < 1e-3:
            rot1 = 0.0
        else:
            rot1 = normalize_angle(np.arctan2(dy, dx) - odom_prev[2])
        rot2 = normalize_angle(odom_curr[2] - odom_prev[2] - rot1)

        # Sample noisy motion components — one draw per particle (vectorized).
        rot1_var = self.a1 * rot1 ** 2 + self.a2 * trans ** 2
        trans_var = self.a3 * trans ** 2 + self.a4 * (rot1 ** 2 + rot2 ** 2)
        rot2_var = self.a1 * rot2 ** 2 + self.a2 * trans ** 2

        rot1_hat = rot1 - sample_normal(rot1_var, size=n)
        trans_hat = trans - sample_normal(trans_var, size=n)
        rot2_hat = rot2 - sample_normal(rot2_var, size=n)

        theta = particles[:, 2]
        new = np.empty_like(particles)
        new[:, 0] = particles[:, 0] + trans_hat * np.cos(theta + rot1_hat)
        new[:, 1] = particles[:, 1] + trans_hat * np.sin(theta + rot1_hat)
        new[:, 2] = normalize_angle(theta + rot1_hat + rot2_hat)
        return new
