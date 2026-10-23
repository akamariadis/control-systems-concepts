# in order to showcase the controller, we simulate a physical plant such as a 1D cart with mass m and damping coefficient b subjected to force u
class MassDamperPlant:
    def __init__(self, mass: float = 1.0, damping: float = 0.5):
        self.mass = mass
        self.b = damping
        self.position = 0.0
        self.velocity = 0.0

    def step(self, force: float, dt: float) -> float:
        # Euler integration
        acceleration = (force - self.b * self.velocity) / self.mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        return self.position
