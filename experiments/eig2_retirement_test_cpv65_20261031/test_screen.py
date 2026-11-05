"""Deterministic controls for CP65's geometry and lifecycle accounting."""
import copy
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import screen as s


def pose(x=0., rot=None):
    p = np.eye(4)
    p[0, 3] = x
    if rot is not None:
        p[:3, :3] = Rotation.from_rotvec(rot).as_matrix()
    return p


class ScreenTests(unittest.TestCase):
    def test_scalar_example(self):
        d = s.decomposition(pose(10.), pose(10.2), pose(10.3))
        self.assertAlmostEqual(d['published_m'], .3)
        self.assertAlmostEqual(d['within_solution_m'], .1)
        self.assertAlmostEqual(d['correction_m'], .2)

    def test_no_revision(self):
        d = s.decomposition(pose(10.), pose(10.), pose(10.1))
        self.assertEqual(d['correction_m'], 0.)
        self.assertEqual(d['within_solution_m'], d['published_m'])

    def test_backward_publication_can_have_forward_within_solve_step(self):
        d = s.decomposition(pose(10.3), pose(10.), pose(10.1))
        self.assertLess(d['published_world_m'][0], 0)
        self.assertGreater(d['within_solution_world_m'][0], 0)

    def test_rotation_order_negative_control(self):
        old = pose(rot=[.2, -.4, .1])
        previous = pose(rot=[-.1, .3, .2])
        current = pose(rot=[.4, .2, -.3])
        s.decomposition(old, previous, current)
        rc = old[:3,:3].T @ previous[:3,:3]
        rm = previous[:3,:3].T @ current[:3,:3]
        rp = old[:3,:3].T @ current[:3,:3]
        self.assertGreater(np.linalg.norm(rp - rm @ rc), .01)

    def test_random_coordinate_equivalence(self):
        rng = np.random.default_rng(65)
        for _ in range(100):
            transforms = []
            for _ in range(4):
                p = pose(rot=rng.normal(size=3))
                p[:3, 3] = rng.normal(size=3)
                transforms.append(p)
            a,b,c,g = transforms
            d = s.decomposition(a,b,c)
            shifted = s.decomposition(g@a,g@b,g@c)
            for field in ('correction_m', 'within_solution_m', 'published_m',
                          'correction_rotation_deg', 'within_solution_rotation_deg', 'published_rotation_deg'):
                self.assertAlmostEqual(d[field], shifted[field], places=10)

    def fixture(self):
        def action(k, edge, kind, details):
            return dict(k=k, edge=edge, action=kind, details=details)
        parsed = dict(life={1:dict(active_factors=0,retiring_factors=0,lag=2,marginal_keys=0),
                            4:dict(active_factors=1,retiring_factors=1,lag=2,marginal_keys=3)},
                      actions=[action(1,'x0->x1','active_window_stage_insert',[]),
                               action(1,'x0->x1','active_window_slot_tracked',['slot=7']),
                               action(4,'x0->x1','active_window_remove_scheduled',['slot=7','slot_valid=true']),
                               action(4,'x3->x4','active_window_stage_insert',[]),
                               action(4,'x3->x4','active_window_slot_tracked',['slot=7'])])
        expected=[dict(k=1,capture_time_s=.2,before=0,after=1,inserted=[dict(edge='x0->x1',slot=7)],removed=[]),
                  dict(k=4,capture_time_s=.8,before=1,after=1,inserted=[dict(edge='x3->x4',slot=7)],
                       removed=[dict(edge='x0->x1',slot=7)])]
        return parsed, expected

    def test_legitimate_slot_reuse_after_retirement(self):
        parsed, expected=self.fixture()
        rows, lifetimes=s.reconcile(parsed, expected)
        self.assertEqual(rows[-1]['after'],1)
        self.assertAlmostEqual(lifetimes[0]['lifetime_s'],.6)

    def test_wrong_removed_slot_rejected(self):
        parsed, expected=self.fixture()
        parsed['actions'][2]['details'][0]='slot=8'
        with self.assertRaises(AssertionError):
            s.reconcile(parsed, expected)

    def test_early_retirement_rejected(self):
        parsed, expected=self.fixture()
        parsed['life'][4]['lag']=5
        with self.assertRaises(AssertionError):
            s.reconcile(parsed, expected)

    def test_direction_is_not_health(self):
        self.assertEqual(s.cosine([1,0,0],[-1,0,0]),-1.)
        self.assertIsNone(s.cosine([0,0,0],[1,0,0]))


if __name__ == '__main__':
    unittest.main()
