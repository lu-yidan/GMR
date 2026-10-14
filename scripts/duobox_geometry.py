"""Opt-in geometric repair: arm/torso clearance and ground-aware foot targets.

Assumes DuoBox's Z-up ground plane is z=0. Contact is a heuristic, not measured.
Does not provide dynamics or pairwise robot collision avoidance.
"""
import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


class GeometryRepair:
    def __init__(self, solver, body_anchor=False):
        self.body_anchor = body_anchor
        self.body_initialized = False
        model = solver.model
        torso_bodies = {model.body(n).id for n in ('torso_link', 'pelvis')}
        # Include fixed descendants: rubber hands are separate child bodies,
        # not geometries owned by wrist_yaw_link.
        elbows = {model.body(side + '_elbow_link').id for side in ('left', 'right')}
        arm_bodies = set()
        for body in range(1, model.nbody):
            ancestor = body
            while ancestor:
                if ancestor in elbows:
                    arm_bodies.add(body)
                    break
                ancestor = int(model.body_parentid[ancestor])
        torso = [g for g in range(model.ngeom) if model.geom_bodyid[g] in torso_bodies and model.geom_group[g] == 1]
        arms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in arm_bodies and model.geom_group[g] == 1]
        self.pairs = [(a, b) for a in arms for b in torso]
        # Visual meshes have zero collision masks; Mink would silently drop them.
        # Enable a dedicated bit on these in-memory IK geometries only.
        model.geom_contype[arms + torso] = 2
        model.geom_conaffinity[arms + torso] = 2
        collision = mink.CollisionAvoidanceLimit(model, [(arms, torso)], gain=.5,
                                minimum_distance_from_collisions=.008,
                                collision_detection_distance=.08)
        assert collision.geom_id_pairs, 'Arm/torso collision pairs must be active'
        solver.ik_limits.append(collision)
        self.previous = {}
        self.frame_hands = None
        self.hand_ids = [model.body(side + '_rubber_hand').id for side in ('left', 'right')]
        self.started = False
        self.target_hands = None
        self.hand_budgets = [(.03, np.deg2rad(10))] * 2

    def begin_frame(self, solver):
        data = solver.configuration.data
        self.frame_hands = [(data.xpos[b].copy(), data.xmat[b].reshape(3, 3).copy())
                            for b in self.hand_ids] if self.started else None
        self.started = True

    def hand_step_valid(self, solver):
        if self.frame_hands is None:
            return True
        data = solver.configuration.data
        for b, (position, rotation), (distance_cap, angle_cap) in zip(self.hand_ids, self.frame_hands, self.hand_budgets):
            if np.linalg.norm(data.xpos[b] - position) > distance_cap + 1e-9:
                return False
            relative = data.xmat[b].reshape(3, 3) @ rotation.T
            if Rotation.from_matrix(relative).magnitude() > angle_cap + 1e-9:
                return False
        return True


    def update_hand_budgets(self, solver):
        targets = [(np.array(solver.scaled_human_data[side + 'Hand'][0], copy=True),
                    Rotation.from_quat(solver.scaled_human_data[side + 'Hand'][1], scalar_first=True))
                   for side in ('Left', 'Right')]
        if self.target_hands is not None:
            self.hand_budgets = [
                (.02 + 1.5 * np.linalg.norm(p - old_p),
                 np.deg2rad(8) + 1.5 * min((r * old_r.inv()).magnitude(),
                     np.arctan2(np.linalg.norm(p - old_p), .3)))
                for (p, r), (old_p, old_r) in zip(targets, self.target_hands)]
        self.target_hands = targets

    def adjust_targets(self, solver, human):
        if self.body_anchor and not self.body_initialized:
            # Start in the actor's facing direction. A 180-degree first-frame
            # turn from the default pose can converge to an inverted IK branch.
            q = solver.configuration.q.copy()
            q[:3], q[3:7] = solver.scaled_human_data['Hips']
            solver.configuration.update(q)
            self.body_initialized = True
        self.update_hand_budgets(solver)
        # Preserve measured body orientation rather than sacrificing the torso
        # to satisfy more heavily weighted foot/end-effector targets.
        for table in ((solver.human_body_to_task1, solver.human_body_to_task2) if self.body_anchor else ()):
            for joint in ('Hips', 'Spine1'):
                if joint in table:
                    table[joint].set_orientation_cost(50.0)
                    if joint == 'Hips':
                        table[joint].set_position_cost(20.0)
        for side in ('Left', 'Right'):
            toe = side + 'ToeBase'
            foot = side + 'Foot'
            raw_pos = np.asarray(human[foot][0])
            raw_toe = np.asarray(human[toe][0])
            rotation = Rotation.from_quat(solver.scaled_human_data[toe][1], scalar_first=True)
            matrix = rotation.as_matrix()
            yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
            flat = Rotation.from_euler('z', yaw)
            # A below-floor toe is invalid even if the human's foot is rotating.
            invalid = raw_toe[2] < -.015
            speed = np.linalg.norm(raw_pos[:2] - self.previous.get(foot, raw_pos)[:2]) * 30
            support = raw_pos[2] < .12 and speed < 1.0
            self.previous[foot] = raw_pos.copy()
            if not (invalid or support):
                continue
            # Preserve planar foot placement. Map the ankle and rigid toe lever,
            # rather than copying a below-floor human toe position.
            pos = raw_pos * solver.human_scale_table['Hips']
            pos[2] = .035
            pos += flat.apply([.1, 0, -.02])
            target = mink.SE3.from_rotation_and_translation(mink.SO3(flat.as_quat(scalar_first=True)), pos)
            for table in (solver.human_body_to_task1, solver.human_body_to_task2):
                if toe in table:
                    table[toe].set_target(target)

    def minimum_distance(self, solver):
        return min(mujoco.mj_geomDistance(solver.model, solver.configuration.data, a, b, .5, None)
                   for a, b in self.pairs)

    def integrate(self, solver, velocity, dt):
        # Linearized collision constraints can still overshoot on a large IK step.
        # Check the actual mesh distances and backtrack before accepting a step.
        old = solver.configuration.q.copy()
        threshold = min(.006, self.minimum_distance(solver)) - 1e-7
        for exponent in range(12):
            candidate = old.copy()
            mujoco.mj_integratePos(solver.model, candidate, velocity, dt * .5 ** exponent)
            solver.configuration.update(candidate)
            if self.minimum_distance(solver) >= threshold and self.hand_step_valid(solver):
                return
        solver.configuration.update(old)
