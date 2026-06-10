"""Demo 2 — plain MCL pose tracking.

The robot drives a loop; MCL is seeded near the true start pose and tracks it.
Prints the localization error over time and (optionally) animates the particles.

Run:
    python data/make_sample_map.py
    python examples/02_run_mcl.py            # text only
    python examples/02_run_mcl.py --plot     # with live animation
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amcl import OccupancyGrid, MonteCarloLocalization, OdometryMotionModel, LikelihoodFieldModel  # noqa: E402
from sim import RobotSimulator  # noqa: E402
from viz import draw_scan  # noqa: E402

DATA = os.path.join(os.path.dirname(__file__), "..", "data", "tutorial_map.npy")


def main(plot=False):
    np.random.seed(0)
    grid = OccupancyGrid.from_numpy(np.load(DATA), resolution=0.05)

    start = np.array([2.0, 2.0, 0.0])
    sim = RobotSimulator(grid, true_pose=start)

    mcl = MonteCarloLocalization(
        grid, num_particles=500,
        motion_model=OdometryMotionModel(alpha=(0.05, 0.05, 0.05, 0.05)),
        sensor_model=LikelihoodFieldModel(grid, sigma_hit=0.2, max_range=8.0),
    )
    mcl.initialize_pose(start, std=(0.3, 0.3, 0.1))

    if plot:
        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots(figsize=(7, 6))

    # A simple rectangular patrol.
    moves = ([(0.2, 0.0)] * 25 + [(0.0, np.pi / 2)] * 1) * 4

    for t, (d_trans, d_rot) in enumerate(moves):
        sim.move(d_trans, d_rot)
        ranges, angles = sim.scan()

        mcl.predict(sim.odom_pose)
        mcl.update(ranges, angles)

        est = mcl.estimate()
        err = np.hypot(*(est[:2] - sim.true_pose[:2]))
        if t % 5 == 0:
            print(f"t={t:3d}  true=({sim.true_pose[0]:.2f},{sim.true_pose[1]:.2f}) "
                  f"est=({est[0]:.2f},{est[1]:.2f})  err={err:.3f} m  Neff={mcl.neff:.0f}")

        if plot:
            ax.clear()
            ax.imshow(grid.grid, origin="lower", cmap="gray_r",
                      extent=[0, grid.width * grid.resolution, 0, grid.height * grid.resolution])
            ax.scatter(mcl.particles[:, 0], mcl.particles[:, 1], s=2, c="tab:blue", alpha=0.3)
            # The observation: laser scan taken from the true pose.
            draw_scan(ax, sim.true_pose, ranges, angles, max_range=sim.max_range)
            ax.plot(sim.true_pose[0], sim.true_pose[1], "g*", ms=15, label="true")
            ax.plot(est[0], est[1], "rx", ms=10, label="estimate")
            ax.legend(loc="upper right")
            ax.set_title(f"MCL  t={t}  err={err:.2f} m")
            plt.pause(0.001)

    print("done.")
    if plot:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main(plot="--plot" in sys.argv)
