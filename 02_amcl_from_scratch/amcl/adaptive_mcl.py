"""Adaptive Monte Carlo Localization (AMCL).

Adds the two ideas that put the "A" in AMCL on top of plain MCL:

1. KLD-sampling   -> adapt the *number* of particles to the spread of the belief.
2. Augmented MCL  -> inject random particles to *recover* from localization
                     failure (bad initial pose, kidnapped robot).

Both live in an overridden ``resample`` step; ``predict`` and ``update`` are
inherited unchanged from ``MonteCarloLocalization``.
"""

from statistics import NormalDist

import numpy as np

from .particle_filter import MonteCarloLocalization
from .utils import normalize_angle


class AdaptiveMCL(MonteCarloLocalization):
    def __init__(self, grid, min_particles=50, max_particles=2000,
                 kld_err=0.05, kld_z=0.99,
                 bin_size=(0.5, 0.5, np.deg2rad(10)),
                 alpha_slow=0.001, alpha_fast=0.1, **kwargs):
        super().__init__(grid, num_particles=max_particles, **kwargs)
        self.min_particles = min_particles
        self.max_particles = max_particles
        self.kld_err = kld_err                       # epsilon
        self.kld_z = NormalDist().inv_cdf(kld_z)     # z_{1-delta}
        self.bin_size = np.asarray(bin_size)         # (dx, dy, dtheta) per KLD bin

        # Augmented MCL running averages of the mean measurement likelihood.
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.w_slow = 0.0
        self.w_fast = 0.0
        self.last_p_random = 0.0  # injection prob used in the most recent resample

    # ---- measurement update tracks w_slow / w_fast -----------------------
    def update(self, ranges, angles):
        # Sharp relative weights drive resampling; the absolute score drives the
        # recovery signal (it keeps its magnitude, so it drops when we're lost).
        likelihood = self.sensor_model.likelihood(self.particles, ranges, angles)
        w_avg = float(np.mean(
            self.sensor_model.absolute_score(self.particles, ranges, angles)))

        self.weights *= likelihood
        total = self.weights.sum()
        if total <= 0 or not np.isfinite(total):
            self.weights[:] = 1.0 / len(self.weights)
        else:
            self.weights /= total

        # Exponential moving averages (initialize on first call).
        if self.w_slow == 0.0:
            self.w_slow = w_avg
        else:
            self.w_slow += self.alpha_slow * (w_avg - self.w_slow)
        if self.w_fast == 0.0:
            self.w_fast = w_avg
        else:
            self.w_fast += self.alpha_fast * (w_avg - self.w_fast)

        # AMCL always resamples (the KLD step decides the new count).
        self.resample()

    # ---- KLD-adaptive resampling with random injection -------------------
    def resample(self):
        weights = self.weights / self.weights.sum()
        cumulative = np.cumsum(weights)
        cumulative[-1] = 1.0

        # Probability of injecting a random particle (augmented MCL recovery).
        if self.w_slow > 0:
            p_random = max(0.0, 1.0 - self.w_fast / self.w_slow)
        else:
            p_random = 0.0
        self.last_p_random = p_random

        free = self.grid.free_cells_world()
        bins = set()
        new_particles = []

        # KLD-sampling: keep drawing until we have enough particles for the
        # number of distinct occupied bins (Fox, 2003).
        m_required = self.min_particles
        while len(new_particles) < m_required:
            if np.random.uniform() < p_random and len(free) > 0:
                # Inject a uniform-random particle on a free cell.
                pt = free[np.random.randint(len(free))]
                particle = np.array([pt[0], pt[1],
                                     np.random.uniform(-np.pi, np.pi)])
            else:
                # Standard importance draw.
                r = np.random.uniform()
                idx = int(np.searchsorted(cumulative, r))
                particle = self.particles[idx].copy()

            new_particles.append(particle)

            # Update the KLD bin count and the required sample size.
            b = tuple(np.floor(particle / self.bin_size).astype(int))
            if b not in bins:
                bins.add(b)
                k = len(bins)
                if k > 1:
                    m_required = self._kld_sample_size(k)
            m_required = int(np.clip(m_required, self.min_particles, self.max_particles))

        self.particles = np.array(new_particles)
        self.particles[:, 2] = normalize_angle(self.particles[:, 2])
        n = len(self.particles)
        self.num_particles = n
        self.weights = np.full(n, 1.0 / n)
        # Note: we deliberately do NOT reset w_fast here. As the injected
        # particles re-localize, w_avg rises, w_fast climbs back toward w_slow,
        # and p_random naturally decays to 0 — the textbook augmented-MCL behavior.

    def _kld_sample_size(self, k):
        """Required #particles for k non-empty bins (Wilson-Hilferty approx)."""
        a = 2.0 / (9.0 * (k - 1))
        m = ((k - 1) / (2.0 * self.kld_err)) * (1 - a + np.sqrt(a) * self.kld_z) ** 3
        return int(np.ceil(m))
