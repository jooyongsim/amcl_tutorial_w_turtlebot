"""Plain Monte Carlo Localization (MCL) — the fixed-particle-count baseline.

This class implements the three-step MCL cycle from the lecture:
    predict (motion) -> update (sensor weights) -> resample.

``AdaptiveMCL`` in ``adaptive_mcl.py`` subclasses this and overrides resampling.
"""

import numpy as np

from .motion_model import OdometryMotionModel
from .measurement_model import LikelihoodFieldModel
from .utils import effective_sample_size, normalize_angle, pose_mean


class MonteCarloLocalization:
    def __init__(self, grid, num_particles=500, motion_model=None,
                 sensor_model=None, resample_threshold=0.5):
        self.grid = grid
        self.num_particles = num_particles
        self.motion_model = motion_model or OdometryMotionModel()
        self.sensor_model = sensor_model or LikelihoodFieldModel(grid)
        # Resample when N_eff drops below this fraction of N.
        self.resample_threshold = resample_threshold

        self.particles = np.zeros((num_particles, 3))
        self.weights = np.full(num_particles, 1.0 / num_particles)
        self._last_odom = None

    # ---- initialization --------------------------------------------------
    def initialize_global(self):
        """Scatter particles uniformly over the free cells of the map."""
        free = self.grid.free_cells_world()
        if len(free) == 0:
            raise ValueError("Map has no free cells to initialize on.")
        idx = np.random.randint(0, len(free), size=self.num_particles)
        self.particles = np.zeros((self.num_particles, 3))
        self.particles[:, :2] = free[idx]
        self.particles[:, 2] = np.random.uniform(-np.pi, np.pi, self.num_particles)
        self.weights[:] = 1.0 / self.num_particles
        self._last_odom = None

    def initialize_pose(self, pose, std=(0.3, 0.3, 0.1)):
        """Seed a tight Gaussian around a known pose (pose tracking)."""
        pose = np.asarray(pose, dtype=float)
        self.particles = np.random.normal(pose, std, size=(self.num_particles, 3))
        self.particles[:, 2] = normalize_angle(self.particles[:, 2])
        self.weights[:] = 1.0 / self.num_particles
        self._last_odom = None

    # ---- the MCL cycle ---------------------------------------------------
    def predict(self, odom):
        """Motion update from the latest odometry pose (x, y, theta)."""
        # Copy: callers (e.g. a simulator) may mutate their odom array in place,
        # which would otherwise make odom_prev and odom_curr the same object.
        odom = np.array(odom, dtype=float)
        if self._last_odom is None:
            self._last_odom = odom
            return
        self.particles = self.motion_model.sample(
            self.particles, self._last_odom, odom)
        self._last_odom = odom

    def update(self, ranges, angles):
        """Measurement update: reweight particles by the sensor likelihood."""
        likelihood = self.sensor_model.likelihood(self.particles, ranges, angles)
        self.weights *= likelihood
        total = self.weights.sum()
        if total <= 0 or not np.isfinite(total):
            # Total collapse — reset to uniform rather than divide by zero.
            self.weights[:] = 1.0 / self.num_particles
        else:
            self.weights /= total
        if effective_sample_size(self.weights) < self.resample_threshold * self.num_particles:
            self.resample()

    def resample(self):
        """Low-variance (systematic) resampling. Resets weights to uniform."""
        self.particles = self._low_variance_resample(self.particles, self.weights)
        self.weights = np.full(self.num_particles, 1.0 / self.num_particles)

    def _low_variance_resample(self, particles, weights):
        n = len(weights)
        positions = (np.arange(n) + np.random.uniform()) / n
        cumulative = np.cumsum(weights)
        cumulative[-1] = 1.0  # guard against fp rounding
        idx = np.searchsorted(cumulative, positions)
        return particles[idx].copy()

    # ---- output ----------------------------------------------------------
    def estimate(self):
        """Current best pose estimate (weighted mean)."""
        return pose_mean(self.particles, self.weights)

    @property
    def neff(self):
        return effective_sample_size(self.weights)
