import unittest
from dataclasses import replace

from food_delivery.navigation import Navigator, Scenario, advance, inverse, wrap


class NavigationTests(unittest.TestCase):
    def test_geodesic_crosses_dateline(self):
        start = (35, 179.999)
        end = advance(start, 0, 200, 1)
        self.assertLess(end[1], 0)
        self.assertAlmostEqual(inverse(start, end)[0], 200, places=5)
        self.assertEqual(wrap(359), -1)

    def test_outage_drifts_and_gps_corrects(self):
        cfg = Scenario()
        nav = Navigator(cfg)
        truth = cfg.origin
        for tick in range(3000):
            # True ground velocity includes a crosswind unknown to the estimator.
            truth = advance(truth, 40, 5, .1)
            nav.update(tick * .1, .1, 40, 0, None)
        self.assertGreater(inverse(truth, nav.position)[0], 1000)
        nav.update(300, .1, 40, 0, truth)
        self.assertLess(inverse(truth, nav.position)[0], 20)

    def test_seed_repeats_measurements(self):
        a, b = Navigator(Scenario()), Navigator(Scenario())
        for tick in range(100):
            fix = Scenario().origin if tick % 10 == 0 else None
            self.assertEqual(a.update(tick * .1, .1, 40, 0, fix),
                             b.update(tick * .1, .1, 40, 0, fix))

    def test_invalid_scenarios(self):
        for cfg in (replace(Scenario(), origin=(95, 0)),
                    replace(Scenario(), duration_s=float('nan')),
                    replace(Scenario(), outage_start_s=900, outage_end_s=100),
                    replace(Scenario(), destination=Scenario().origin)):
            with self.assertRaises(ValueError):
                cfg.validate()


if __name__ == '__main__':
    unittest.main()
