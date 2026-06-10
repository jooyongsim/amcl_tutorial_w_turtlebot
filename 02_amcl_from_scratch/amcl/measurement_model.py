"""Likelihood-field range-finder sensor model.

For each laser beam we project its *endpoint* into the map, look up the distance
to the nearest obstacle (precomputed distance transform on the grid), and score
it with a Gaussian. This is the default model in ``nav2_amcl`` because it is fast
(no per-beam ray casting at runtime) and produces a smooth likelihood surface.

The model is fully vectorized over (particles x beams).
"""

import numpy as np


class LikelihoodFieldModel:
    def __init__(self, grid, sigma_hit=0.2, z_hit=0.85, z_rand=0.15,
                 max_range=10.0, max_beams=60):
        """
        grid       : OccupancyGrid (provides the distance field).
        sigma_hit  : std-dev (m) of the Gaussian around an obstacle.
        z_hit      : weight of the "correct measurement" component.
        z_rand     : weight of the uniform "random" component.
        max_range  : sensor max range (m); beams at/over this are ignored.
        max_beams  : subsample the scan to at most this many beams (speed + avoids
                     overconfidence from treating correlated beams as independent).
        """
        self.grid = grid
        self.sigma_hit = sigma_hit
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.max_range = max_range
        self.max_beams = max_beams

    def _subsample(self, ranges, angles):
        n = len(ranges)
        if n <= self.max_beams:
            idx = np.arange(n)
        else:
            idx = np.linspace(0, n - 1, self.max_beams).astype(int)
        return ranges[idx], angles[idx]

    def log_likelihood(self, particles, ranges, angles):
        """Core computation. Returns ``(log_w, n_beams)``.

        ``log_w`` is the (N,) array of summed log per-beam probabilities
        (NOT yet max-normalized), and ``n_beams`` is how many valid beams were
        used. Both ``likelihood`` (relative weights) and the AMCL recovery
        signal (absolute score) are derived from this.
        """
        particles = np.asarray(particles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float)
        ranges, angles = self._subsample(ranges, angles)

        # Keep only valid beams (a max-range reading carries no endpoint info).
        valid = np.isfinite(ranges) & (ranges < self.max_range) & (ranges > 0)
        ranges, angles = ranges[valid], angles[valid]
        if len(ranges) == 0:
            return np.zeros(len(particles)), 0

        px = particles[:, 0][:, None]      # (N, 1)
        py = particles[:, 1][:, None]
        pth = particles[:, 2][:, None]

        # World-frame beam endpoints for every (particle, beam) pair -> (N, B).
        beam_world = pth + angles[None, :]
        ex = px + ranges[None, :] * np.cos(beam_world)
        ey = py + ranges[None, :] * np.sin(beam_world)

        dist = self.grid.nearest_obstacle_distance(ex.ravel(), ey.ravel())
        dist = dist.reshape(ex.shape)
        dist = np.where(np.isfinite(dist), dist, 3.0 * self.sigma_hit)

        # Per-beam probability: Gaussian hit + uniform random component.
        p_hit = np.exp(-(dist ** 2) / (2.0 * self.sigma_hit ** 2))
        p_beam = self.z_hit * p_hit + self.z_rand / self.max_range

        log_w = np.sum(np.log(p_beam + 1e-12), axis=1)
        return log_w, len(ranges)

    def likelihood(self, particles, ranges, angles):
        """Return an (N,) array of *relative* weights p(scan | particle, map).

        Max-normalized for numerical stability — only ratios between particles
        are meaningful, which is all the resampling step needs.
        """
        log_w, n = self.log_likelihood(particles, ranges, angles)
        if n == 0:
            return np.ones(len(particles))
        log_w = log_w - log_w.max()  # stabilize before exponentiating
        return np.exp(log_w)

    def absolute_score(self, particles, ranges, angles):
        """Return an (N,) array of *absolute* per-particle scores in (0, 1].

        The geometric mean of per-beam probabilities: ``exp(mean_b log p_beam)``.
        Unlike :meth:`likelihood` this keeps its absolute magnitude, so a sudden
        drop signals localization failure — exactly what augmented MCL needs.
        """
        log_w, n = self.log_likelihood(particles, ranges, angles)
        if n == 0:
            return np.ones(len(particles))
        return np.exp(log_w / n)
