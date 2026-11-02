import math
import unittest

import numpy as np

from robotics.contracts import VehicleState
from robotics.flight_controllers.visual_follow import VisualFollower


class VisualFollowTests(unittest.TestCase):
    def test_world_image_error_rotates_into_body_command(self):
        yaw = math.pi/2
        state = VehicleState(np.array([0., 0., 9.]),
                             np.array([0., 0., math.sin(yaw/2), math.cos(yaw/2)]),
                             np.zeros(3), np.zeros(3))
        command = VisualFollower().update(state, (400,220,440,260), (640,480), 0)
        self.assertAlmostEqual(command.forward_mps, 0, places=10)
        self.assertLess(command.right_mps, 0)

    def test_reacquisition_does_not_use_motion_from_before_loss(self):
        state = VehicleState(np.array([0., 0., 9.]), np.array([0., 0., 0., 1.]),
                             np.zeros(3), np.zeros(3))
        follower = VisualFollower()
        follower.update(state, (320,220,360,260), (640,480), 0)
        follower.update(state, (360,220,400,260), (640,480), .1)
        follower.update(state, None, (640,480), .2)
        command = follower.update(state, (310,230,330,250), (640,480), .3)
        self.assertEqual(command.forward_mps, 0)
        self.assertEqual(command.right_mps, 0)


if __name__ == '__main__':
    unittest.main()
