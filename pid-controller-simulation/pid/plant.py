class Cart1DPlant:
    """Simulates a 1D cart with mass and viscous friction."""

    def __init__(
        self,
        mass: float = 1.0,
        damping: float = 0.5,
        initial_position: float = 0.0,
        initial_velocity: float = 0.0,
    ):
        if mass <= 0.0:
            raise ValueError("Mass must be strictly positive.")
        self.mass = float(mass)
        self.damping = float(damping)
        self.position = float(initial_position)
        self.velocity = float(initial_velocity)

    def _derivatives(self, pos: float, vel: float, force: float) -> tuple[float, float]:
        """Compute state derivatives: dx/dt = vel, d(vel)/dt = (force - b*vel)/m."""
        acceleration = (force - self.damping * vel) / self.mass
        return vel, acceleration

    def step(self, force: float, dt: float) -> float:
        """Advance the cart dynamics by dt using 4th-order Runge-Kutta integration."""
        x0 = self.position
        v0 = self.velocity

        # RK4 stages
        k1_x, k1_v = self._derivatives(x0, v0, force)
        k2_x, k2_v = self._derivatives(x0 + 0.5 * dt * k1_x, v0 + 0.5 * dt * k1_v, force)
        k3_x, k3_v = self._derivatives(x0 + 0.5 * dt * k2_x, v0 + 0.5 * dt * k2_v, force)
        k4_x, k4_v = self._derivatives(x0 + dt * k3_x, v0 + dt * k3_v, force)

        # Update states
        self.position += (dt / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x)
        self.velocity += (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)

        return self.position

    def reset(self, position: float = 0.0, velocity: float = 0.0) -> None:
        """Reset cart position and velocity states."""
        self.position = float(position)
        self.velocity = float(velocity)
