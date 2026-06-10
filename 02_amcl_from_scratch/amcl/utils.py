"""Small math helpers shared across the from-scratch AMCL modules."""

import numpy as np


def normalize_angle(theta):
    """Wrap angle(s) to the range (-pi, pi].

    Works on scalars and numpy arrays.
    """
    return np.arctan2(np.sin(theta), np.cos(theta))


def sample_normal(variance, size=None):
    """Draw zero-mean Gaussian noise given a *variance* (not std-dev).

    The motion model is parameterized in terms of variances, so this wrapper
    keeps the call sites readable: ``sample_normal(alpha * d**2)``.
    """
    std = np.sqrt(np.maximum(variance, 0.0))
    return np.random.normal(0.0, std, size=size)


def pose_mean(particles, weights=None):
    """Weighted mean pose of a particle set.

    ``particles`` is an (N, 3) array of (x, y, theta). The heading is averaged
    on the unit circle so that, e.g., the mean of +179 deg and -179 deg is 180,
    not 0.
    """
    if weights is None:
        weights = np.full(len(particles), 1.0 / len(particles))
    weights = weights / weights.sum()
    x = np.sum(weights * particles[:, 0])
    y = np.sum(weights * particles[:, 1])
    c = np.sum(weights * np.cos(particles[:, 2]))
    s = np.sum(weights * np.sin(particles[:, 2]))
    theta = np.arctan2(s, c)
    return np.array([x, y, theta])


def effective_sample_size(weights):
    """N_eff = 1 / sum(w_i^2). Low values signal particle degeneracy."""
    w = weights / weights.sum()
    return 1.0 / np.sum(w ** 2)
