import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from franka_hand_teleop.core import Limits
from franka_hand_teleop.workspace_guard import WorkspaceGuard


class GuardTests(unittest.TestCase):
    def test_batch_matches_scalar_fk_for_rotated_bodies_slides_and_full_finger_envelope(self):
        urdf = '''<robot><link name="arm"><collision><origin xyz="-.05 0 0" rpy=".2 .1 .4"/>
        <geometry><box size=".3 .15 .15"/></geometry></collision></link>
        <joint name="swing" type="revolute"><parent link="fr3_link0"/><child link="arm"/>
        <origin xyz=".3 0 .3"/><axis xyz="0 0 2"/><limit lower="-2" upper="2"/></joint>
        <link name="wrist"><collision><geometry><cylinder radius=".04" length=".1"/>
        </geometry></collision></link><joint name="slide" type="prismatic"><parent link="arm"/>
        <child link="wrist"/><origin xyz=".1 0 .1" rpy=".2 .1 .3"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="1"/></joint><joint name="tool" type="fixed"><parent link="wrist"/>
        <child link="fr3_hand_tcp"/><origin xyz=".15 0 .05"/></joint>
        <link name="fr3_finger"><collision><geometry><box size=".04 .02 .1"/></geometry>
        </collision></link><joint name="finger" type="prismatic"><parent link="wrist"/>
        <child link="fr3_finger"/><origin xyz="0 .02 0"/><axis xyz="0 1 0"/>
        <limit lower="0" upper=".04"/></joint></robot>'''
        guard = WorkspaceGuard(urdf, Limits(minimum=(-.2, -.35, 0), maximum=(.9, .35, 1),
                                            body_minimum=(-.3, -.4, -.1), body_maximum=(1, .4, 1.2)))
        groups = {True: [], False: []}
        for angle in (-1.4, -.6, 0., .6, 1.4):
            for slide in (0., .2, .4, .6):
                values = (angle, slide)
                state = dict(zip(guard.names, values))
                expected = guard.inside(guard.tcp(state)) and guard.body_inside(state)[0]
                self.assertEqual(guard.samples_inside([values]), expected)
                groups[expected].append(values)
        self.assertTrue(groups[True])
        self.assertTrue(groups[False])
        self.assertTrue(guard.samples_inside(groups[True]))
        self.assertFalse(guard.samples_inside(groups[True]+groups[False]))
        self.assertFalse(guard.samples_inside([]))
        self.assertFalse(guard.samples_inside([(0., float("nan"))]))

    def test_elbow_collision_geometry_can_block_motion_while_tcp_stays_inside(self):
        urdf = '''<robot><link name="arm"><collision><origin xyz="-.2 0 0"/>
        <geometry><box size=".6 .1 .1"/></geometry></collision></link>
        <joint name="swing" type="revolute"><parent link="fr3_link0"/><child link="arm"/>
        <origin xyz="0 0 .4"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/>
        </joint><joint name="tool" type="fixed"><parent link="arm"/>
        <child link="fr3_hand_tcp"/><origin xyz=".4 0 0"/></joint></robot>'''
        limits = Limits(minimum=(.25, -.2, .15), maximum=(.6, .2, .65),
                        body_minimum=(-.55, -.1, -.05), body_maximum=(.6, .1, .65))
        guard = WorkspaceGuard(urdf, limits)
        self.assertTrue(guard.body_inside({"swing": 0})[0])
        self.assertTrue(guard.inside(guard.tcp({"swing": .2})))
        self.assertFalse(guard.body_inside({"swing": .2})[0])
        valid, reason = guard.check({"swing": 0}, [[.2]])
        self.assertFalse(valid)
        self.assertIn("whole-arm", reason)

    def test_rejects_outside_waypoint_before_execution(self):
        urdf = '''<robot><joint name="slide" type="prismatic">
        <parent link="fr3_link0"/><child link="fr3_hand_tcp"/>
        <origin xyz="0 0 .4"/><axis xyz="1 0 0"/><limit lower="0" upper="1"/>
        </joint></robot>'''
        guard = WorkspaceGuard(urdf, Limits(minimum=(.25, -.2, .15), maximum=(.6, .2, .65)))
        self.assertEqual(guard.tcp({"slide": .4}), (.4, 0, .4))
        self.assertTrue(guard.check({"slide": .4}, [[.5]])[0])
        self.assertFalse(guard.check({"slide": .4}, [[.7]])[0])
        self.assertFalse(guard.check({"slide": .4}, [[float("nan")]])[0])

    def test_rejects_curved_joint_path_even_when_both_endpoints_are_inside(self):
        urdf = '''<robot><joint name="swing" type="revolute">
        <parent link="fr3_link0"/><child link="arm"/>
        <origin xyz="0 0 .4"/><axis xyz="0 0 1"/><limit lower="-2" upper="2"/>
        </joint><joint name="tool" type="fixed"><parent link="arm"/>
        <child link="fr3_hand_tcp"/><origin xyz=".6 0 0"/></joint></robot>'''
        guard = WorkspaceGuard(urdf, Limits(maximum=(.5, .5, .65), minimum=(.25, -.5, .15)))
        self.assertTrue(guard.inside(guard.tcp({"swing": -.8})))
        self.assertTrue(guard.inside(guard.tcp({"swing": .8})))
        self.assertFalse(guard.check({"swing": -.8}, [[.8]])[0])


if __name__ == "__main__":
    unittest.main()
