"""Plotting helpers for the demos — visualizing the laser observation.

Kept separate from the filter code so the algorithm stays uncluttered.
"""

import numpy as np


def draw_scan(ax, pose, ranges, angles, max_range=8.0, color="red",
              draw_beams=True, beam_stride=6, label="laser scan (observation)"):
    """Draw a laser scan taken from ``pose`` (x, y, theta).

    Endpoints are plotted as dots (where each beam hit), and — optionally —
    faint lines from the sensor to each endpoint show the beams themselves.
    Beams at/over ``max_range`` (no return) are skipped.

    Parameters
    ----------
    ax        : matplotlib Axes to draw on.
    pose      : (3,) sensor pose in world coords (x, y, theta).
    ranges    : (B,) measured beam distances.
    angles    : (B,) beam angles in the sensor frame.
    beam_stride : draw every Nth beam line (dots are always all drawn) to
                  avoid a cluttered fan.
    """
    x, y, th = pose
    ranges = np.asarray(ranges)
    angles = np.asarray(angles)
    valid = np.isfinite(ranges) & (ranges < max_range) & (ranges > 0)
    a = th + angles[valid]
    ex = x + ranges[valid] * np.cos(a)
    ey = y + ranges[valid] * np.sin(a)

    if draw_beams:
        for i in range(0, len(ex), beam_stride):
            ax.plot([x, ex[i]], [y, ey[i]], color=color, lw=0.4, alpha=0.2)

    ax.scatter(ex, ey, s=5, c=color, alpha=0.8, edgecolors="none", label=label)


def draw_expected_scan(ax, grid, pose, angles, max_range=8.0,
                       color="orange", label="expected scan (from estimate)"):
    """Draw the scan a robot *would* see from ``pose`` by ray casting the map.

    Useful next to ``draw_scan`` to show what the measurement model compares the
    real observation against — when the estimate is right, the two overlap.
    """
    x, y, th = pose
    ex, ey = [], []
    for ang in angles:
        r = grid.ray_cast(x, y, th + ang, max_range)
        if r < max_range:
            ex.append(x + r * np.cos(th + ang))
            ey.append(y + r * np.sin(th + ang))
    ax.scatter(ex, ey, s=5, marker="x", c=color, alpha=0.7, label=label)
