"""From-scratch AMCL on Occupancy Grid Maps.

A small, dependency-light (numpy/scipy) implementation built for teaching. Each
module maps directly onto a lecture topic:

    occupancy_grid    -> slides/01  (the map)
    motion_model      -> slides/04  (prediction)
    measurement_model -> slides/04  (correction)
    particle_filter   -> slides/03  (Monte Carlo Localization)
    adaptive_mcl      -> slides/05  (KLD-sampling + augmented MCL)
"""

from .occupancy_grid import OccupancyGrid
from .motion_model import OdometryMotionModel
from .measurement_model import LikelihoodFieldModel
from .particle_filter import MonteCarloLocalization
from .adaptive_mcl import AdaptiveMCL

__all__ = [
    "OccupancyGrid",
    "OdometryMotionModel",
    "LikelihoodFieldModel",
    "MonteCarloLocalization",
    "AdaptiveMCL",
]
