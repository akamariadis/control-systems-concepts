import numpy as np
import matplotlib.pyplot as plt
from pid.controller import PIDController
from pid.plant import MassDamperPlant

def run_simulation(kp=20.0, ki=10.0, kd=4.0, duration=10.0, dt=0.01):
    pid = PIDController(kp, ki, kd, output_limits=(-100.0, 100.0))
    plant = MassDamperPlant(mass=1.0, damping=0.5)

    time_steps = np.arange(0, duration, dt)
    setpoint = 1.0  # Step input to 1.0 m

    positions = []
    control_signals = []

    for _ in time_steps:
        u = pid.update(setpoint, plant.position, dt)
        pos = plant.step(u, dt)
        positions.append(pos)
        control_signals.append(u)

    # Plot results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(time_steps, positions, label="Response $x(t)$", color="#1f77b4", lw=2)
    ax1.axhline(setpoint, color="red", linestyle="--", label="Setpoint")
    ax1.set_ylabel("Position (m)")
    ax1.grid(True)
    ax1.legend()

    ax2.plot(time_steps, control_signals, label="Control Effort $u(t)$", color="#ff7f0e")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force (N)")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.savefig("assets/step_response.png", dpi=150)
    plt.show()

if __name__ == "__main__":
    run_simulation()
