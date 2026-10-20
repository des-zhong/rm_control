import math
import unittest
from as2_follow.core import Limits, Tracker, select_team


class TrackingTests(unittest.TestCase):
    def test_team_identity_locked_by_caller(self):
        poses = [(0., 0., 0.)] + [(x, 0., 0.) for x in (-2., 1., -1., 2., -.5, .5)]
        team = select_team(poses, 'lower')
        self.assertEqual(team, (1, 3, 5))
        self.assertEqual(select_team(poses, 'upper'), (2, 4, 6))
        poses[1] = (2., 0., 0.)
        self.assertEqual(team, (1, 3, 5))
        with self.assertRaises(ValueError):
            select_team(poses, 'lower')

    def test_odometry_alignment(self):
        t = Tracker((0., -2.5, math.pi/2), Limits())
        self.assertEqual(t.world_pose((10., 20., 0.)), (0., -2.5, math.pi/2))
        p = t.world_pose((10.1, 20., .1))
        self.assertAlmostEqual(p[0], 0.)
        self.assertAlmostEqual(p[1], -2.4)
        self.assertAlmostEqual(p[2], math.pi/2+.1)

    def test_sideways_while_facing_ball(self):
        t = Tracker((0., -2.5, math.pi/2), Limits())
        vx, vy, wz = t.command((0., -2.5, math.pi/2), 1., (0., 0.), .05)
        self.assertAlmostEqual(vx, 0.)
        self.assertLess(vy, 0.)
        self.assertAlmostEqual(wz, 0.)
        self.assertLessEqual(math.hypot(vx, vy), .004001)

    def test_y_feedback_and_speed_bound(self):
        t = Tracker((0., -2.5, 0.), Limits())
        for _ in range(100):
            cmd = t.command((0., -2.4, 0.), 100., (0., 0.), .05)
            self.assertLessEqual(math.hypot(*cmd[:2]), .120001)
            self.assertLessEqual(abs(cmd[2]), .250001)
        self.assertLess(cmd[1], 0.)
        self.assertEqual(t.stop(), (0., 0., 0.))

    def test_jump_and_corridor_stop(self):
        t = Tracker((0., -2.5, 0.), Limits())
        t.world_pose((0., 0., 0.))
        with self.assertRaises(ValueError):
            t.world_pose((1., 0., 0.))
        with self.assertRaises(ValueError):
            t.command((0., -2., 0.), 0., (0., 0.), .05)
        with self.assertRaises(ValueError):
            t.command((3., -2.5, 0.), 0., (0., 0.), .05)

    def test_upper_wrap_and_x_limit(self):
        t = Tracker((2.3, 2.5, -math.pi/2), Limits())
        cmd = t.command((2.3, 2.5, -math.pi/2), 3., (2.3, 0.), .05)
        self.assertEqual(cmd, (0., 0., 0.))
        cmd = t.command((2.3, 2.5, math.pi-.01), 2.3, (0., 2.2), .05)
        self.assertGreater(cmd[2], 0.)


if __name__ == '__main__':
    unittest.main()
