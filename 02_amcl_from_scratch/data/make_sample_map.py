"""Generate a small synthetic Occupancy Grid Map for the tutorials.

Creates a rectangular room with a perimeter wall and a couple of interior
obstacles, then saves it as a ``.npy`` array (used by the Python demos) and,
if Pillow is available, a ROS-style ``.pgm`` + ``.yaml`` pair.

Run:  python data/make_sample_map.py
"""

import os

import numpy as np

RESOLUTION = 0.05      # meters per cell
ORIGIN = (0.0, 0.0)    # world coords of cell (0, 0)


def build_grid():
    """Return a (H, W) occupancy array: 0=free, 1=occupied."""
    h, w = 200, 240  # 10 m x 12 m at 0.05 m/cell
    grid = np.zeros((h, w), dtype=float)

    # Perimeter walls (3 cells thick).
    grid[:3, :] = 1.0
    grid[-3:, :] = 1.0
    grid[:, :3] = 1.0
    grid[:, -3:] = 1.0

    # An interior dividing wall with a doorway.
    grid[60:63, 30:150] = 1.0
    grid[60:63, 90:110] = 0.0  # doorway gap

    # A couple of box obstacles.
    grid[120:150, 60:90] = 1.0
    grid[100:130, 170:200] = 1.0

    return grid


def save_npy(grid, path):
    np.save(path, grid)
    print(f"wrote {path}  shape={grid.shape}  resolution={RESOLUTION}")


def save_ros_map(grid, stem):
    """Write ``stem.pgm`` + ``stem.yaml`` in the ROS map_server format."""
    try:
        from PIL import Image
    except ImportError:
        print("Pillow not installed -> skipping .pgm/.yaml export")
        return
    # ROS pgm: white=free(255), black=occupied(0); row 0 is top (max y).
    img = np.where(grid >= 0.65, 0, 254).astype(np.uint8)
    img = np.flipud(img)
    Image.fromarray(img, mode="L").save(stem + ".pgm")
    with open(stem + ".yaml", "w") as f:
        f.write(
            f"image: {os.path.basename(stem)}.pgm\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
    print(f"wrote {stem}.pgm and {stem}.yaml")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    grid = build_grid()
    save_npy(grid, os.path.join(here, "tutorial_map.npy"))
    save_ros_map(grid, os.path.join(here, "tutorial_map"))
