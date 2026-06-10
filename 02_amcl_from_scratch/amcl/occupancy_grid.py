"""Occupancy Grid Map (OGM) container and the operations localization needs.

The grid is stored as a 2D numpy array indexed ``grid[row, col]`` where:
    0.0  -> free
    1.0  -> occupied
    0.5  -> unknown

Row increases with +y (world), col increases with +x (world). ``origin`` is the
world coordinate of the cell (row=0, col=0).
"""

import numpy as np


class OccupancyGrid:
    def __init__(self, grid, resolution, origin=(0.0, 0.0)):
        self.grid = np.asarray(grid, dtype=float)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)  # (x0, y0)
        self.height, self.width = self.grid.shape
        self._distance_field = None  # lazily computed (meters to nearest obstacle)

    # ---- world <-> grid conversion ---------------------------------------
    def world_to_grid(self, x, y):
        """Return integer (col, row) for world point (x, y). Vectorized."""
        col = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        return col, row

    def grid_to_world(self, col, row):
        """Return world (x, y) of a cell *center*."""
        x = self.origin[0] + (np.asarray(col) + 0.5) * self.resolution
        y = self.origin[1] + (np.asarray(row) + 0.5) * self.resolution
        return x, y

    def in_bounds(self, col, row):
        return (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)

    def is_occupied(self, x, y, thresh=0.65):
        col, row = self.world_to_grid(x, y)
        inside = self.in_bounds(col, row)
        col = np.clip(col, 0, self.width - 1)
        row = np.clip(row, 0, self.height - 1)
        occ = self.grid[row, col] >= thresh
        # Treat out-of-bounds as occupied (you can't be outside the map).
        return np.where(inside, occ, True)

    def is_free(self, x, y, free_thresh=0.25):
        col, row = self.world_to_grid(x, y)
        inside = self.in_bounds(col, row)
        col = np.clip(col, 0, self.width - 1)
        row = np.clip(row, 0, self.height - 1)
        free = self.grid[row, col] <= free_thresh
        return np.where(inside, free, False)

    # ---- ray casting (beam model / map prediction) -----------------------
    def ray_cast(self, x, y, theta, max_range, step=None):
        """Distance to the first occupied cell along a beam from (x, y, theta).

        Simple fixed-step DDA. Returns ``max_range`` if nothing is hit. Scalar
        inputs only (used for the beam model and visualization).
        """
        step = step or (self.resolution * 0.5)
        d = 0.0
        cx, cy = float(x), float(y)
        dx, dy = np.cos(theta) * step, np.sin(theta) * step
        while d < max_range:
            cx += dx
            cy += dy
            d += step
            if bool(self.is_occupied(cx, cy)):
                return d
        return max_range

    # ---- likelihood field (distance transform) ---------------------------
    @property
    def distance_field(self):
        """Per-cell distance (meters) to the nearest occupied cell.

        Computed once and cached. Used by the likelihood-field sensor model.
        """
        if self._distance_field is None:
            self._distance_field = self._compute_distance_field()
        return self._distance_field

    def _compute_distance_field(self):
        occupied = self.grid >= 0.65
        if not occupied.any():
            return np.full(self.grid.shape, np.inf)
        try:
            from scipy.ndimage import distance_transform_edt
            # EDT gives distance (in cells) to the nearest *zero* pixel, so we
            # invert: zeros where occupied, ones elsewhere.
            dist_cells = distance_transform_edt(~occupied)
        except Exception:
            dist_cells = self._bfs_distance(occupied)
        return dist_cells * self.resolution

    def _bfs_distance(self, occupied):
        """Fallback multi-source BFS if scipy is unavailable (8-connectivity)."""
        from collections import deque

        dist = np.full(occupied.shape, np.inf)
        dq = deque()
        rows, cols = np.where(occupied)
        for r, c in zip(rows, cols):
            dist[r, c] = 0.0
            dq.append((r, c))
        while dq:
            r, c = dq.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < occupied.shape[0] and 0 <= nc < occupied.shape[1]:
                        nd = dist[r, c] + np.hypot(dr, dc)
                        if nd < dist[nr, nc]:
                            dist[nr, nc] = nd
                            dq.append((nr, nc))
        return dist

    def nearest_obstacle_distance(self, x, y):
        """Look up the distance field at world points (vectorized)."""
        col, row = self.world_to_grid(x, y)
        inside = self.in_bounds(col, row)
        col = np.clip(col, 0, self.width - 1)
        row = np.clip(row, 0, self.height - 1)
        d = self.distance_field[row, col]
        # Out-of-bounds endpoints are treated as "far from any obstacle".
        return np.where(inside, d, np.inf)

    def free_cells_world(self, free_thresh=0.25):
        """World (x, y) centers of all free cells — used for global init."""
        rows, cols = np.where(self.grid <= free_thresh)
        x, y = self.grid_to_world(cols, rows)
        return np.column_stack([x, y])

    # ---- IO --------------------------------------------------------------
    @classmethod
    def from_numpy(cls, array, resolution, origin=(0.0, 0.0)):
        return cls(array, resolution, origin)

    @classmethod
    def from_ros_yaml(cls, yaml_path):
        """Load a ROS map (``.yaml`` + ``.pgm``). Requires PyYAML + Pillow."""
        import os
        import yaml
        from PIL import Image

        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img_path = meta["image"]
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        img = np.array(Image.open(img_path).convert("L"), dtype=float) / 255.0
        # In ROS pgm: white(1.0)=free, black(0.0)=occupied. Convert to occupancy.
        negate = int(meta.get("negate", 0))
        p_occ = img if negate else (1.0 - img)
        occ_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.196))
        grid = np.full(p_occ.shape, 0.5)
        grid[p_occ >= occ_thresh] = 1.0
        grid[p_occ <= free_thresh] = 0.0
        # pgm row 0 is the top (max y); flip so row increases with +y.
        grid = np.flipud(grid)
        origin = tuple(meta.get("origin", [0.0, 0.0])[:2])
        return cls(grid, float(meta["resolution"]), origin)
