import unittest
import mink
import mujoco
import numpy as np
from duobox_continuity import PreviousFrameLimit


class FrameBudgetTest(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body><freejoint/><geom type="sphere" size=".1"/>
            <body pos="0 0 .2"><joint name="hinge" range="-2 2"/>
              <geom type="sphere" size=".1"/></body>
          </body></worldbody></mujoco>''')
        self.configuration = mink.Configuration(self.model)
        self.limit = PreviousFrameLimit(self.model, max_speed=6, fps=30)

    def test_internal_iterations_share_one_output_frame_budget(self):
        self.limit.previous = self.configuration.q.copy()
        q = self.configuration.q.copy()
        q[7] = .15
        self.configuration.update(q)
        constraints = self.limit.compute_qp_inequalities(self.configuration, .002)
        np.testing.assert_allclose(constraints.h, [.05, .35], atol=1e-12)
        q[7] = .19
        self.configuration.update(q)
        later = self.limit.compute_qp_inequalities(self.configuration, .01)
        np.testing.assert_allclose(later.h, [.01, .39], atol=1e-12)
        np.testing.assert_array_equal(later.G[:, :6], 0)

    def test_first_frame_is_not_constrained_to_default_pose(self):
        self.assertIsNone(self.limit.compute_qp_inequalities(self.configuration, .002).G)


class GeometryRepairTest(unittest.TestCase):
    def test_active_mesh_constraints_and_backtracking(self):
        from pathlib import Path
        from types import SimpleNamespace
        from duobox_geometry import GeometryRepair
        model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1] /
                                              'assets/unitree_g1/g1_mocap_29dof.xml'))
        solver = SimpleNamespace(model=model, configuration=mink.Configuration(model), ik_limits=[])
        repair = GeometryRepair(solver)
        self.assertTrue(solver.ik_limits[-1].geom_id_pairs)
        active = set(solver.ik_limits[-1].geom_id_pairs)
        head = next(g for g in range(model.ngeom) if model.geom_group[g] == 1
                    and model.geom_dataid[g] >= 0 and model.mesh(model.geom_dataid[g]).name == 'head_link')
        for side in ('left', 'right'):
            hand = next(g for g in range(model.ngeom) if model.geom_group[g] == 1
                        and model.geom_bodyid[g] == model.body(side + '_rubber_hand').id)
            self.assertIn((hand, head), repair.pairs)
            self.assertTrue((hand, head) in active or (head, hand) in active)
        solver.scaled_human_data = {side + 'Hand': [np.zeros(3), [1, 0, 0, 0]]
                                    for side in ('Left', 'Right')}
        repair.update_hand_budgets(solver)
        solver.scaled_human_data['RightHand'][0] = np.array([.15, 0, 0])
        repair.update_hand_budgets(solver)
        self.assertAlmostEqual(repair.hand_budgets[0][0], .02)
        self.assertAlmostEqual(repair.hand_budgets[1][0], .245)
        repair.update_hand_budgets(solver)
        self.assertAlmostEqual(repair.hand_budgets[1][0], .02)
        initial = solver.configuration.q.copy()
        initial_distance = repair.minimum_distance(solver)
        self.assertGreater(initial_distance, .006)
        repair.begin_frame(solver)
        repair.begin_frame(solver)
        # Large joint steps stress the nonlinear integration safeguard.
        rng = np.random.default_rng(4)
        for _ in range(15):
            velocity = rng.normal(size=model.nv) * 30
            velocity[:6] = 0
            repair.integrate(solver, velocity, .1)
            self.assertGreaterEqual(repair.minimum_distance(solver), .006 - 2e-6)
            self.assertTrue(repair.hand_step_valid(solver))


if __name__ == '__main__':
    unittest.main()
