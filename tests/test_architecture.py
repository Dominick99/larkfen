from __future__ import annotations

import inspect
import unittest

import numpy as np

from companion_computer import BoundingBoxInterceptionGuidance
from drone_protocol import BoundingBoxObservation, VehicleState
from flight_controller import QuadcopterFlightController


class ArchitectureBoundaryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.state = VehicleState(
            position=np.array([0.0, 0.0, 2.5]),
            orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
            timestamp_seconds=1.0,
        )

    def test_guidance_and_flight_control_exchange_shared_contracts(self) -> None:
        observation = BoundingBoxObservation(280, 160, 360, 240, 640, 368, 1.0)
        setpoint = BoundingBoxInterceptionGuidance().update(observation, self.state)
        wrench = QuadcopterFlightController().update(self.state, setpoint)

        self.assertEqual(setpoint.timestamp_seconds, 1.0)
        self.assertTrue(np.isfinite(setpoint.position).all())
        self.assertTrue(np.isfinite(wrench.world_force).all())
        self.assertTrue(np.isfinite(wrench.body_torque).all())

    def test_deployable_components_do_not_import_pybullet(self) -> None:
        self.assertNotIn("pybullet", inspect.getsource(BoundingBoxInterceptionGuidance))
        self.assertNotIn("pybullet", inspect.getsource(QuadcopterFlightController))


if __name__ == "__main__":
    unittest.main()
