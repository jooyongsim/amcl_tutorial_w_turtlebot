# AMCL from scratch (pure Python)

A dependency-light implementation of **Monte Carlo Localization** and **Adaptive
MCL** on an Occupancy Grid Map — every step written by hand with `numpy`. No ROS.
This is the track for *understanding the algorithm*; pair it with the slides.

## What's inside

```
amcl/
├── occupancy_grid.py     # the OGM: world↔grid, ray casting, distance field   (slides/01)
├── motion_model.py       # probabilistic odometry motion model (prediction)   (slides/04)
├── measurement_model.py  # likelihood-field sensor model (correction)         (slides/04)
├── particle_filter.py    # MonteCarloLocalization: predict→update→resample    (slides/03)
└── adaptive_mcl.py       # AdaptiveMCL: KLD-sampling + augmented MCL recovery  (slides/05)

data/
└── make_sample_map.py    # generate a synthetic room map (.npy + ROS .pgm/.yaml)

examples/
├── sim.py                # tiny ground-truth robot sim (odometry + laser scans)
├── viz.py                # plotting helpers — draw the laser scan (observation)
├── 01_show_map.py        # visualize the map and its likelihood field
├── 02_run_mcl.py         # MCL pose tracking
└── 03_run_amcl.py        # AMCL global localization + kidnapped-robot recovery

tests/
└── test_particle_filter.py
```

## Setup

```bash
cd 02_amcl_from_scratch
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Run the demos

```bash
# 1. Create the sample map (writes data/tutorial_map.npy + .pgm/.yaml)
python data/make_sample_map.py

# 2. Look at the map and its distance transform
python examples/01_show_map.py

# 3. Plain MCL tracking a known start pose  (--plot for live particles)
python examples/02_run_mcl.py --plot

# 4. AMCL: global localization + recovery after a kidnap  (--plot to watch)
python examples/03_run_amcl.py --plot
```

In the `--plot` window you'll see: the **map** (gray), the **particle cloud**
(blue), the **true pose** (green star), the **estimate** (red ✕), and the
**laser scan / observation** (red dots + faint beams) cast from the true pose.
As the filter converges, the scan endpoints sit on the map walls and the
estimate meets the true pose — the visual version of "the laser matches the map."

## Run the tests

```bash
pytest                              # if you have pytest
python tests/test_particle_filter.py   # or run directly, no pytest needed
```

## How the pieces map to the algorithm

| Lecture step | Code |
|--------------|------|
| Represent belief with particles | `MonteCarloLocalization.particles`, `.weights` |
| **Predict** (motion) | `predict()` → `OdometryMotionModel.sample()` |
| **Correct** (sensor) | `update()` → `LikelihoodFieldModel.likelihood()` |
| **Resample** | `_low_variance_resample()` (systematic) |
| Adaptive particle count | `AdaptiveMCL._kld_sample_size()` (KLD-sampling) |
| Failure recovery | `AdaptiveMCL` `w_slow`/`w_fast` random injection |

## Suggested exercises

1. Swap the likelihood field for a **beam model** (use `OccupancyGrid.ray_cast`).
2. Make the sensor noise too small in `02_run_mcl.py` and watch **particle deprivation**.
3. Plot `amcl.num_particles` over time in demo 3 — see KLD shrink then grow after the kidnap.
4. Tune `alpha_fast`/`alpha_slow` and observe recovery speed vs. false injections.
