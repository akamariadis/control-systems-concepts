import unittest
from pid.controller import PIDController
from pid.plant import Cart1DPlant


class TestPIDController(unittest.TestCase):
    def test_proportional_action(self):
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0)
        output = pid.update(setpoint=2.0, measured_value=0.0, dt=0.1)
        # error = 2.0 -> output = 10.0 * 2.0 = 20.0
        self.assertAlmostEqual(output, 20.0)

    def test_integral_accumulation(self):
        pid = PIDController(kp=0.0, ki=5.0, kd=0.0)
        # Step 1: error = 1.0, dt = 0.1 -> integral = 0.1 -> output = 0.5
        out1 = pid.update(setpoint=1.0, measured_value=0.0, dt=0.1)
        self.assertAlmostEqual(out1, 0.5)
        # Step 2: error = 1.0, dt = 0.1 -> integral = 0.2 -> output = 1.0
        out2 = pid.update(setpoint=1.0, measured_value=0.0, dt=0.1)
        self.assertAlmostEqual(out2, 1.0)

    def test_derivative_kick_mitigation(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=2.0)
        # First step shouldn't calculate derivative to avoid impulse jump
        out_first = pid.update(setpoint=5.0, measured_value=0.0, dt=0.1)
        self.assertAlmostEqual(out_first, 0.0)

        # Second step: error stays 5.0 -> d_error/dt = 0
        out_steady = pid.update(setpoint=5.0, measured_value=0.0, dt=0.1)
        self.assertAlmostEqual(out_steady, 0.0)

    def test_output_saturation_and_anti_windup(self):
        pid = PIDController(kp=10.0, ki=10.0, kd=0.0, output_limits=(-15.0, 15.0))
        # Large error forces saturation
        output = pid.update(setpoint=10.0, measured_value=0.0, dt=1.0)
        self.assertEqual(output, 15.0)

        # Anti-windup check: integral accumulator should not increase unboundedly
        self.assertLessEqual(pid._integral, 10.0)


class TestCartPlant(unittest.TestCase):
    def test_free_deceleration(self):
        cart = Cart1DPlant(mass=2.0, damping=1.0, initial_position=0.0, initial_velocity=10.0)
        # Without external force, velocity must strictly decrease
        cart.step(force=0.0, dt=0.1)
        self.assertLess(cart.velocity, 10.0)
        self.assertGreater(cart.position, 0.0)


if __name__ == "__main__":
    unittest.main()
