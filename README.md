# AMCL on Occupancy Grid Maps — Tutorial & Lectures

Course materials for **AMCL** (Adaptive Monte Carlo Localization) on **OGM** (Occupancy Grid Maps),
covering both the theory and two complementary implementations.

> Note: "ACML" in the course title refers to **AMCL** — Adaptive Monte Carlo Localization,
> the particle-filter localization algorithm used to estimate a robot's pose on an occupancy grid map.

## Repository layout

| Folder | What it contains |
|--------|------------------|
| [`slides/`](slides/) | Lecture slides (Markdown / Marp). Theory of OGM, Bayes filtering, MCL, and AMCL. |
| [`01_amcl_with_packages/`](01_amcl_with_packages/) | **AMCL + navigation using existing packages** — ROS 2 + Nav2. You configure and launch; the heavy lifting is done by the `nav2_amcl` package. |
| [`02_amcl_from_scratch/`](02_amcl_from_scratch/) | **AMCL + navigation from scratch** — pure Python (numpy/matplotlib). Every component of the particle filter is implemented by hand for teaching. |

## Suggested learning path

1. Read the slides in order (`slides/00_*` → `slides/06_*`).
2. Run the **from-scratch** demos (`02_amcl_from_scratch/`) to internalize the math — no ROS required.
3. Run the **ROS 2 / Nav2** package tutorial (`01_amcl_with_packages/`) to see the production-grade stack.

## Prerequisites

- **From scratch:** Python 3.9+, `numpy`, `scipy`, `matplotlib`.
- **With packages:** ROS 2 (Humble or Jazzy), Nav2, a simulator (Gazebo / TurtleBot3) or recorded bag.

See each subfolder's `README.md` for detailed setup and run instructions.
