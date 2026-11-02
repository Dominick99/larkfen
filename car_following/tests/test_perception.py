import unittest

import numpy as np

from car_following.perception import ColorDetector, Detection, GreenTracker, iou
from robotics.contracts import VehicleState
from robotics.flight_controllers.visual_follow import VisualFollower


class PerceptionTests(unittest.TestCase):
    def test_rgb_selection_ignores_red_and_blue(self):
        rgb = np.zeros((120, 240, 3), dtype=np.uint8)
        rgb[20:50, 20:60] = [240, 10, 10]
        rgb[20:50, 100:140] = [10, 220, 20]
        rgb[20:50, 180:220] = [10, 20, 240]
        detections = ColorDetector().detect(rgb)
        self.assertEqual({d.label for d in detections}, {'red', 'blue', 'green'})
        track = GreenTracker().update(detections, 0)
        self.assertEqual(track.detection.box, (100,20,140,50))

    def test_lost_track_has_no_stale_box_and_reacquires(self):
        tracker = GreenTracker()
        green = Detection((20,20,40,40), 'green', 1)
        first = tracker.update([green], 0)
        lost = tracker.update([], 1)
        self.assertIsNone(lost.detection)
        self.assertEqual(lost.state, 'lost')
        self.assertEqual(tracker.update([green], 2).track_id, first.track_id)
        tracker.update([], 6)
        self.assertNotEqual(tracker.update([green], 7).track_id, first.track_id)

    def test_no_box_brakes_and_preserves_altitude(self):
        state = VehicleState(np.array([0.,0.,9.]), np.array([0.,0.,0.,1.]),
                             np.array([2.,1.,0.]), np.zeros(3))
        follower = VisualFollower()
        command = follower.update(state, (390,200,430,240), (640,480), 0)
        self.assertGreater(command.forward_mps, 0)
        self.assertGreater(command.right_mps, 0)
        lost = follower.update(state, None, (640,480), .1)
        self.assertEqual((lost.forward_mps,lost.right_mps,lost.up_mps), (0,0,0))

    def test_iou_handles_missing_and_overlap(self):
        self.assertEqual(iou(None,(0,0,10,10)),0)
        self.assertEqual(iou((0,0,10,10),(0,0,10,10)),1)
        self.assertAlmostEqual(iou((0,0,10,10),(5,0,15,10)),1/3)


if __name__ == '__main__':
    unittest.main()
