"""Demo 1 — load the sample map and visualize it (plus its distance field).

Run:
    python data/make_sample_map.py      # once, to create the map
    python examples/01_show_map.py
"""

import os
import sys

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amcl import OccupancyGrid  # noqa: E402

DATA = os.path.join(os.path.dirname(__file__), "..", "data", "tutorial_map.npy")


def main():
    grid = OccupancyGrid.from_numpy(np.load(DATA), resolution=0.05)
    print(f"map: {grid.width} x {grid.height} cells @ {grid.resolution} m/cell")
    print(f"free cells: {len(grid.free_cells_world())}")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    ax1.imshow(grid.grid, origin="lower", cmap="gray_r")
    ax1.set_title("Occupancy Grid Map")
    im = ax2.imshow(grid.distance_field, origin="lower", cmap="viridis")
    ax2.set_title("Likelihood field (distance to nearest obstacle)")
    fig.colorbar(im, ax=ax2, label="meters")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
