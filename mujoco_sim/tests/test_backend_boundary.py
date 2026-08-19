from __future__ import annotations

import inspect
import unittest

from mujoco_sim.robots import Quadcopter


class MuJoCoBackendBoundaryTests(unittest.TestCase):
    def test_mujoco_dependency_stays_in_backend(self) -> None:
        source = inspect.getsource(Quadcopter)

        self.assertIn("mj_objectVelocity", source)
        self.assertIn("RotorModel", source)


if __name__ == "__main__":
    unittest.main()
