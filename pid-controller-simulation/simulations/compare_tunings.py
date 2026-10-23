import numpy as np
import matplotlib.pyplot as plt
from pid.controller import PIDController
from pid.plant import Cart1DPlant


def compare_tunings():
    t_end = 8.0
    dt = 0.01
    time_steps = np.arange(0, t_end, dt)
    setpoint = 1.0

    tuning_profiles = [
        {"name": "P-Only (Kp=15, Ki=0, Kd=0)", "kp": 15.0, "ki": 0.0, "kd": 0.0, "color": "#d62728"},
        {"name": "Underdamped (Kp=40, Ki=20, Kd=1)", "kp": 40.0, "ki": 20.0, "kd": 1.0, "color": "#ff7f0e"},
        {"name": "Overdamped (Kp=12, Ki=2, Kd=8)", "kp": 12.0, "ki": 2.0, "kd": 8.0, "color": "#2ca02c"},
        {"name": "Well-Tuned (Kp=28, Ki=14, Kd=6)", "kp": 28.0, "ki": 14.0, "kd": 6.0, "color": "#1f77b4"},
    ]

    fig, (ax_pos, ax_force) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    for profile in tuning_profiles:
        pid = PIDController(
            kp=profile["kp"],
            ki=profile["ki"],
            kd=profile["kd"],
            output_limits=(-60.0, 60.0),
        )
        cart = Cart1DPlant(mass=1.2, damping=0.8)

        pos_history = []
        force_history = []

        for _ in time_steps:
            force = pid.update(setpoint, cart.position, dt)
            pos = cart.step(force, dt)
            pos_history.append(pos)
            force_history.append(force)

        ax_pos.plot(time_steps, pos_history, label=profile["name"], color=profile["color"], lw=1.8)
        ax_force.plot(time_steps, force_history, label=profile["name"], color=profile["color"], alpha=0.8)

    ax_pos.axhline(setpoint, color="black", linestyle="--", lw=1.2, label="Setpoint")
    ax_pos.set_ylabel("Position (m)")
    ax_pos.set_title("1D Cart Response Across Different PID Tunings")
    ax_pos.grid(True, linestyle="--", alpha=0.5)
    ax_pos.legend(loc="lower right")

    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel("Control Force (N)")
    ax_force.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    plt.savefig("tuning_comparison.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    compare_tunings()
