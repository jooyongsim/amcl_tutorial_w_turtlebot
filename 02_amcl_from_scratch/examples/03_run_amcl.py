"""Demo 3 — AMCL global localization + kidnapped-robot recovery.

Shows the two things plain MCL can't do well:
  1. **Global localization** — particles start scattered over the whole map and
     collapse onto the true pose (watch the adaptive particle count shrink).
  2. **Recovery** — halfway through we "kidnap" the robot (teleport it). Augmented
     MCL detects the likelihood drop and injects random particles to recover.

Run:
    python data/make_sample_map.py
    python examples/03_run_amcl.py
    python examples/03_run_amcl.py --plot
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amcl import OccupancyGrid, AdaptiveMCL, OdometryMotionModel, LikelihoodFieldModel  # noqa: E402
from sim import RobotSimulator  # noqa: E402
from viz import draw_scan  # noqa: E402

DATA = os.path.join(os.path.dirname(__file__), "..", "data", "tutorial_map.npy")


def main(plot=False):
    np.random.seed(1)
    grid = OccupancyGrid.from_numpy(np.load(DATA), resolution=0.05)

    start = np.array([2.0, 2.0, 0.0])
    sim = RobotSimulator(grid, true_pose=start)

    amcl = AdaptiveMCL(
        grid, min_particles=100, max_particles=3000,
        motion_model=OdometryMotionModel(alpha=(0.08, 0.08, 0.08, 0.08)),
        sensor_model=LikelihoodFieldModel(grid, sigma_hit=0.2, max_range=8.0),
        # Faster averaging than the textbook (0.001/0.1) so w_slow reaches
        # steady state within this short demo and the recovery clearly fires.
        alpha_slow=0.05, alpha_fast=0.3,
    )
    amcl.initialize_global()  # no idea where we are
    print(f"start: global localization with {amcl.num_particles} particles")

    if plot:
        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots(figsize=(7, 6))

    # Long enough that w_slow settles before the kidnap, then plenty of steps
    # for the injected particles to re-localize afterward.
    moves = [(0.2, 0.0)] * 25 + [(0.0, np.pi / 2)] + [(0.2, 0.0)] * 50
    kidnap_at = 35

    for t, (d_trans, d_rot) in enumerate(moves):
        sim.move(d_trans, d_rot)

        if t == kidnap_at:
            sim.true_pose = np.array([8.0, 8.0, np.pi])  # teleport!
            print(f"--- t={t}: ROBOT KIDNAPPED to (8.0, 8.0) ---")

        ranges, angles = sim.scan()
        amcl.predict(sim.odom_pose)
        amcl.update(ranges, angles)

        est = amcl.estimate()
        err = np.hypot(*(est[:2] - sim.true_pose[:2]))
        if t % 3 == 0 or t == kidnap_at:
            print(f"t={t:3d}  N={amcl.num_particles:4d}  err={err:.2f} m  "
                  f"p_inject={amcl.last_p_random:.2f}")

        if plot:
            ax.clear()
            ax.imshow(grid.grid, origin="lower", cmap="gray_r",
                      extent=[0, grid.width * grid.resolution, 0, grid.height * grid.resolution])
            ax.scatter(amcl.particles[:, 0], amcl.particles[:, 1], s=2, c="tab:blue", alpha=0.2)
            # The observation: laser scan taken from the true pose.
            draw_scan(ax, sim.true_pose, ranges, angles, max_range=sim.max_range)
            ax.plot(sim.true_pose[0], sim.true_pose[1], "g*", ms=15, label="true")
            ax.plot(est[0], est[1], "rx", ms=10, label="estimate")
            ax.legend(loc="upper right")
            ax.set_title(f"AMCL  t={t}  N={amcl.num_particles}  err={err:.2f} m")
            plt.pause(0.001)

    print("done.")
    if plot:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main(plot="--plot" in sys.argv)
