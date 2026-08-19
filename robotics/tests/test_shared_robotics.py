from __future__ import annotations

import inspect
import unittest

import numpy as np

from robotics.flight_controllers import SimpleVelocityFlightController
from robotics.robot_models.quadcopter import (
    QuadcopterConfig,
    RotorModel,
    allocate_rotor_throttle,
)


class SharedRoboticsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = QuadcopterConfig()

    def test_hover_allocation_is_balanced(self) -> None:
        throttle = allocate_rotor_throttle(
            self.config.mass_kg * 9.81,
            np.zeros(3),
            self.config,
        )
        thrust = self.config.max_thrust_per_rotor_n * throttle**2

        self.assertTrue(np.allclose(throttle, throttle[0]))
        self.assertAlmostEqual(float(thrust.sum()), self.config.mass_kg * 9.81)

    def test_motor_response_moves_toward_command(self) -> None:
        motors = RotorModel(self.config)
        first = motors.step(np.ones(4), 1.0 / 240.0)
        later = first
        for _ in range(100):
            later = motors.step(np.ones(4), 1.0 / 240.0)

        self.assertTrue(np.all(first > 0.0))
        self.assertTrue(np.all(later > first))
        self.assertTrue(np.allclose(later, self.config.max_thrust_per_rotor_n))

    def test_shared_code_has_no_simulator_imports(self) -> None:
        sources = "\n".join(
            (
                inspect.getsource(SimpleVelocityFlightController),
                inspect.getsource(RotorModel),
                inspect.getsource(allocate_rotor_throttle),
            )
        )

        self.assertNotIn("pybullet", sources.lower())
        self.assertNotIn("mujoco", sources.lower())


if __name__ == "__main__":
    unittest.main()
