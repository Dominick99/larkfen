from __future__ import annotations

import inspect
import unittest

from pybullet_sim.robots import Quadcopter


class PyBulletBackendBoundaryTests(unittest.TestCase):
    def test_pybullet_dependency_stays_in_backend(self) -> None:
        source = inspect.getsource(Quadcopter)

        self.assertIn("p.createMultiBody", source)
        self.assertIn("RotorModel", source)


if __name__ == "__main__":
    unittest.main()
