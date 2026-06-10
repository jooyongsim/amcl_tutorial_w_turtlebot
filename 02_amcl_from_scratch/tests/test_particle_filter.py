"""Lightweight sanity tests for the from-scratch AMCL building blocks.

Run with either:
    pytest
    python tests/test_particle_filter.py     # no pytest required
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amcl import (OccupancyGrid, OdometryMotionModel, LikelihoodFieldModel,  # noqa: E402
                  MonteCarloLocalization, AdaptiveMCL)
from amcl.utils import normalize_angle, pose_mean  # noqa: E402


def _toy_grid():
    grid = np.zeros((40, 40))
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 1.0
    return OccupancyGrid.from_numpy(grid, resolution=0.05)


def test_normalize_angle():
    assert np.isclose(normalize_angle(0.0), 0.0)
    assert np.isclose(normalize_angle(2 * np.pi + 0.3), 0.3)
    # +-3*pi sits on the boundary: -pi and +pi are equivalent, so compare on
    # the unit circle rather than the raw value.
    for a in (3 * np.pi, -3 * np.pi):
        assert np.isclose(abs(normalize_angle(a)), np.pi)


def test_world_grid_roundtrip():
    grid = _toy_grid()
    col, row = grid.world_to_grid(1.0, 0.5)
    x, y = grid.grid_to_world(col, row)
    # Recovered point must fall in the same cell.
    assert grid.world_to_grid(x, y) == (col, row)


def test_motion_model_advances_forward():
    mm = OdometryMotionModel(alpha=(0, 0, 0, 0))  # noise-free
    particles = np.zeros((10, 3))
    new = mm.sample(particles, np.array([0, 0, 0]), np.array([1.0, 0, 0]))
    assert np.allclose(new[:, 0], 1.0, atol=1e-6)
    assert np.allclose(new[:, 1], 0.0, atol=1e-6)


def test_distance_field_zero_on_walls():
    grid = _toy_grid()
    # A wall cell should have ~zero distance to the nearest obstacle.
    d = grid.nearest_obstacle_distance(np.array([0.0]), np.array([0.5]))
    assert d[0] < grid.resolution * 2


def test_resampling_preserves_count():
    grid = _toy_grid()
    mcl = MonteCarloLocalization(grid, num_particles=200)
    mcl.initialize_pose([1.0, 1.0, 0.0])
    mcl.weights = np.random.rand(200)
    mcl.weights /= mcl.weights.sum()
    mcl.resample()
    assert len(mcl.particles) == 200
    assert np.allclose(mcl.weights, 1.0 / 200)


def test_kld_sample_size_grows_with_bins():
    grid = _toy_grid()
    amcl = AdaptiveMCL(grid, min_particles=10, max_particles=5000)
    assert amcl._kld_sample_size(50) > amcl._kld_sample_size(5)


def test_mcl_tracks_a_known_pose():
    """A static robot with good sensing should keep low error."""
    np.random.seed(0)
    grid = _toy_grid()
    sensor = LikelihoodFieldModel(grid, sigma_hit=0.15, max_range=4.0)
    mcl = MonteCarloLocalization(grid, num_particles=300, sensor_model=sensor)
    true = np.array([1.0, 1.0, 0.0])
    mcl.initialize_pose(true, std=(0.2, 0.2, 0.1))

    # Fake a scan consistent with the true pose by ray casting from it.
    angles = np.linspace(-np.pi, np.pi, 60, endpoint=False)
    ranges = np.array([grid.ray_cast(true[0], true[1], true[2] + a, 4.0) for a in angles])

    for _ in range(5):
        mcl.update(ranges, angles)
    est = mcl.estimate()
    assert np.hypot(*(est[:2] - true[:2])) < 0.3


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS  {fn.__name__}")
    print(f"\nAll {len(fns)} tests passed.")


if __name__ == "__main__":
    _run_all()
