import numpy as np
import matplotlib.pyplot as plt
from pid.controller import PIDController
from pid.plant import Cart1DPlant


def simulate_step(
    kp: float = 25.0,
    ki: float = 12.0,
    kd: float = 6.0,
    setpoint: float = 1.0,
    duration: float = 8.0,
    dt: float = 0.01,
):
    pid = PIDController(kp=kp, ki=ki, kd=kd, output_limits=(-50.0, 50.0))
    plant = Cart1DPlant(mass=1.2, damping=0.8)

    time_steps = np.arange(0, duration, dt)
    positions = []
    velocities = []
    control_efforts = []

    for _ in time_steps:
        u = pid.update(setpoint, plant.position, dt)
        pos = plant.step(u, dt)
        positions.append(pos)
        velocities.append(plant.velocity)
        control_efforts.append(u)

    positions = np.array(positions)
    control_efforts = np.array(control_efforts)

    # Compute response metrics
    overshoot = max(0.0, (np.max(positions) - setpoint) / setpoint * 100.0)
    print(f"Final Position: {positions[-1]:.4f} m")
    print(f"Max Overshoot:  {overshoot:.2f}%")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    ax1.plot(time_steps, positions, label="Cart Position $x(t)$", color="#1f77b4", lw=2)
    ax1.axhline(setpoint, color="red", linestyle="--", label="Target Setpoint")
    ax1.set_ylabel("Position (m)")
    ax1.set_title("1D Cart Closed-Loop Step Response")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="lower right")

    ax2.plot(time_steps, control_efforts, label="Force $u(t)$", color="#ff7f0e", lw=1.5)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Applied Force (N)")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper right")

    plt.tight_layout()
    plt.savefig("step_response.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    simulate_step()
