"""Frame-level continuity limits for DuoBox IK, in source time (30 Hz)."""
import mink
import numpy as np
from mink.limits import Constraint, Limit


class PreviousFrameLimit(Limit):
    """Bound accumulated IK displacement from the previous output frame.

    Unlike a per-iteration velocity limit, repeated internal IK iterations cannot
    spend the displacement budget multiple times. Floating base is not limited.
    """
    def __init__(self, model, max_speed=8.0, fps=30):
        self.previous = None
        self.max_delta = max_speed / fps
        self.projection = np.eye(model.nv)[6:]

    def compute_qp_inequalities(self, configuration, dt):
        if self.previous is None:
            return Constraint()
        current = configuration.q[7:]
        lo = self.previous[7:] - self.max_delta
        hi = self.previous[7:] + self.max_delta
        return Constraint(G=np.vstack((self.projection, -self.projection)),
                          h=np.concatenate((hi-current, current-lo)))


def add_continuity(solver, max_speed=8.0):
    limit = PreviousFrameLimit(solver.model, max_speed)
    posture = mink.PostureTask(solver.model, cost=0.2)
    posture.set_target_from_configuration(solver.configuration)
    solver.ik_limits.append(limit)
    solver.tasks1.append(posture)
    solver.tasks2.append(posture)
    return limit, posture


def solve_frame(solver, human_data, geometry=None):
    """GMR's two stages with limits passed by keyword for current mink APIs."""
    if geometry is not None:
        geometry.begin_frame(solver)
    solver.update_targets(human_data)
    if geometry is not None:
        geometry.adjust_targets(solver, human_data)
    dt = solver.model.opt.timestep
    for enabled, tasks in [(solver.use_ik_match_table1, solver.tasks1),
                           (solver.use_ik_match_table2, solver.tasks2)]:
        if not enabled:
            continue
        def error():
            return np.linalg.norm(np.concatenate([task.compute_error(solver.configuration) for task in tasks]))
        for _ in range(solver.max_iter + 1):
            before = error()
            velocity = mink.solve_ik(solver.configuration, tasks, dt, solver.solver,
                                     damping=solver.damping, limits=solver.ik_limits)
            if geometry is None:
                solver.configuration.integrate_inplace(velocity, dt)
            else:
                geometry.integrate(solver, velocity, dt)
            if before - error() <= .001:
                break
    return solver.configuration.q.copy()
