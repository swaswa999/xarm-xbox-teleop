"""Offline tests; every robot command goes to FakeArm."""
from dataclasses import replace
import math
import struct
import threading
import unittest
from unittest import mock
import xarm_xbox as x
from test_xarm_teleop import FakeArm as KeyboardFake, Clock
from xarm_teleop import ReportWatch


class FakeArm(KeyboardFake):
    tcp_acc_limit = [1.0, 50000.0]
    world_offset = [0.0] * 6
    collision_sensitivity = 3
    soft_stop_state = 5

    @property
    def error_code(self): return self.error
    @property
    def warn_code(self): return self.warning
    @property
    def position(self): return self.pose[:]

    def set_position(self, *pose, **kw):
        self.calls.append(('position', pose, kw))
        return 0

    def set_position_aa(self, pose, **kw):
        self.calls.append(('rotation', pose, kw))
        return 0

    def set_state(self, state):
        result = super().set_state(state)
        if state == 6:
            self.state = self.soft_stop_state  # Observed mode-7 result on this robot.
        return result


class XboxTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.arm = FakeArm()
        self.c = x.XboxTeleop(self.arm, clock=self.clock)
        self.controls = x.PadControls()
        self.frame()  # Initial connection must not activate held buttons.

    def frame(self, xyz=(0, 0, 0), grip=0, buttons=(), focused=True,
              connected=True, precision=False, rxyz=(0, 0, 0), **kw):
        pad = x.Pad(connected=connected, name='Fake Xbox', xyz=xyz, grip=grip,
                    precision=precision, buttons=frozenset(buttons), focused=focused,
                    updated=self.clock(), rxyz=rxyz)
        result = self.controls.frame(self.c, pad, **kw)
        self.c.tick()
        return result

    def arm_on(self):
        self.frame(buttons={'menu'})
        self.frame()
        self.assertTrue(self.c.armed, self.c.message)
        self.arm.calls.clear()

    def moves(self):
        return [c for c in self.arm.calls if c[0] in ('position', 'rotation')]

    def test_startup_disarmed_and_connection_with_held_menu_never_enable(self):
        self.frame(xyz=(1, 0, 0))
        self.c.close()
        self.assertEqual(self.arm.calls, [])
        c = x.XboxTeleop(FakeArm(), clock=self.clock)
        k = x.PadControls()
        k.frame(c, x.Pad(True, buttons=frozenset({'menu'}), focused=True, updated=self.clock()))
        c.tick()
        self.assertEqual(c.arm.calls, [])

    def test_enable_mode7_without_home_or_position_move(self):
        self.frame(buttons={'menu'})
        self.assertTrue(self.c.armed)
        self.assertIn(('mode', 7), self.arm.calls)
        self.assertEqual(self.moves(), [])
        self.assertFalse(any(c[0] == 'velocity' for c in self.arm.calls))

    def test_enable_requires_neutral_motion_controls_but_ignores_a_l3(self):
        for fields in [dict(xyz=(.1, 0, 0)), dict(rxyz=(0, .1, 0)), dict(grip=1)]:
            self.frame(**fields, buttons={'menu'})
            self.assertEqual(self.arm.calls, [])
            self.frame()
        self.frame(buttons={'menu', 'a', 'l3'})
        self.assertTrue(self.c.armed)
        self.assertEqual(self.moves(), [])

    def test_mode_changed_state_requires_explicit_enable(self):
        self.arm.state = 5
        self.frame(xyz=(1, 0, 0))
        self.assertEqual(self.arm.calls, [])
        self.frame()
        self.arm_on()
        self.arm.state = 5
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertEqual(self.moves(), [])

    def test_axes_move_directly_after_enable_and_centering_stops(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.assertEqual(len(self.moves()), 1)
        self.frame()
        self.assertEqual(self.arm.calls[-1], ('state', 6))
        self.assertTrue(self.c.armed)

    def test_mode7_targets_are_bounded_and_use_continuous_planning(self):
        self.arm_on()
        for _ in range(20):
            self.clock.advance()
            self.frame(xyz=(1, 1, 1))
        self.assertEqual(len(self.moves()), 20)
        for _, pose, opts in self.moves():
            distance = math.sqrt(sum((pose[i]-self.arm.pose[i])**2 for i in range(3)))
            self.assertAlmostEqual(distance, 60)
            self.assertEqual(pose[3:], tuple(self.arm.pose[3:]))
            self.assertAlmostEqual(opts['speed'], 230 * math.sqrt((1 + 1 + .75**2) / 3))
            self.assertEqual(opts['mvacc'], 1000)
            self.assertFalse(opts['wait'])

    def test_analog_and_precision_speed(self):
        self.arm_on()
        self.frame(xyz=(0.5, 0, 0))
        self.assertEqual(self.moves()[-1][2]['speed'], 115)
        self.clock.advance()
        self.frame(xyz=(0.5, 0, 0), precision=True)
        self.assertEqual(self.moves()[-1][2]['speed'], 23)

    def test_z_bumpers_are_25_percent_slower_in_both_directions(self):
        for button, sign in [('lb', -1), ('rb', 1)]:
            self.setUp()
            self.arm_on()
            pad = x.native_pad(dict(version=2, connected=True, axes=[0]*6,
                                    buttons=[button]), self.clock())
            self.controls.frame(self.c, replace(pad, focused=True))
            self.c.tick()
            _, target, options = self.moves()[-1]
            self.assertEqual(target[:2], tuple(self.arm.pose[:2]))
            self.assertEqual(target[2] - self.arm.pose[2], sign * 60)
            self.assertEqual(options['speed'], 172.5)
            self.frame()
            self.assertEqual(self.arm.calls[-1], ('state', 6))

    def test_z_scaling_preserves_xy_components_in_combined_moves(self):
        self.arm_on()
        self.frame(xyz=(1, -1, 1))
        _, target, options = self.moves()[-1]
        delta = [target[i] - self.arm.pose[i] for i in range(3)]
        length = math.sqrt(sum(v*v for v in delta))
        velocity = [v / length * options['speed'] for v in delta]
        before = 230 / math.sqrt(3)
        self.assertAlmostEqual(velocity[0], before)
        self.assertAlmostEqual(velocity[1], -before)
        self.assertAlmostEqual(velocity[2], before * .75)
        self.assertLessEqual(options['speed'], 230)

    def test_z_scaling_preserves_precision_and_reduced_limits(self):
        self.arm.is_reduced_mode = True
        self.arm.reduced_max_tcp_speed = 80
        self.arm_on()
        self.frame(xyz=(0, 0, 1), precision=True)
        self.assertAlmostEqual(self.moves()[-1][2]['speed'], 80 * .75 * .2)

    def test_release_stops_once_using_native_deceleration(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.frame(xyz=(1, 0, 0))
        self.frame()
        self.assertEqual(self.arm.calls[-1], ('state', 6))
        self.assertEqual(self.arm.calls.count(('state', 6)), 1)
        self.assertTrue(self.c.armed)
        self.clock.advance()
        self.frame(xyz=(-1, 0, 0))
        self.assertLess(self.moves()[-1][1][0], self.arm.pose[0])

    def test_controller_disconnect_and_focus_loss_stop_and_require_reenable(self):
        for reason in ({'connected': False}, {'focused': False}, {'lost': True}):
            with self.subTest(reason=reason):
                self.setUp()
                self.arm_on()
                self.frame(xyz=(1, 0, 0))
                self.frame(xyz=(1, 0, 0), **reason)
                self.assertFalse(self.c.armed)
                self.assertEqual(self.arm.calls[-1], ('state', 4))
                count = len(self.moves())
                self.frame(xyz=(1, 0, 0))
                self.assertEqual(len(self.moves()), count)

    def test_lost_heartbeat_stops_and_external_faults_cannot_auto_resume(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.clock.advance(0.2)
        self.c.tick()
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.assertFalse(self.c.armed)
        self.frame()
        self.arm_on()
        self.arm.error = 19
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertEqual(self.moves(), [])

    def test_controller_sample_inside_150ms_limit_is_not_a_stale_window(self):
        self.arm_on()
        self.clock.advance(.12)
        self.c.publish_pad(replace(self.c.pad, xyz=(1, 0, 0), updated=self.clock() - .12))
        self.c.tick()
        self.assertTrue(self.c.armed, self.c.message)
        self.assertEqual(len(self.moves()), 1)
        self.assertNotIn('a', self.c.pad.buttons)
        self.assertEqual(self.c.input.updated, self.clock())

    def test_fresh_window_cannot_hide_stale_controller_sample(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.clock.advance(.16)
        self.c.publish_pad(replace(self.c.pad, updated=self.clock() - .16))
        self.c.tick()
        self.assertFalse(self.c.armed)
        self.assertIn("Controller input stopped", self.c.message)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.frame(xyz=(1, 0, 0))
        self.assertEqual(len(self.moves()), 1)

    def test_100ms_window_watchdog_still_stops_before_controller_timeout(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.clock.advance(.11)
        self.c.tick()
        self.assertFalse(self.c.armed)
        self.assertIn("Control window updates stopped", self.c.message)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.frame(xyz=(1, 0, 0))
        self.assertEqual(len(self.moves()), 1)

    def test_stop_button_wins_over_enable(self):
        self.frame(buttons={'view', 'menu'})
        self.assertEqual(self.arm.calls, [])
        self.frame()
        self.arm_on()
        self.frame(xyz=(1, 0, 0), buttons={'view'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.moves(), [])

    def test_reduced_limits_preserved(self):
        self.arm.is_reduced_mode = True
        self.arm.reduced_max_tcp_speed = 50
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.assertEqual(self.moves()[-1][2]['speed'], 50)
        self.arm.reduced_max_tcp_speed = 30
        self.clock.advance()
        self.frame(xyz=(1, 0, 0))
        self.assertEqual(self.moves()[-1][2]['speed'], 30)

    def test_gripper_requires_y_then_works_directly_and_never_moves_arm(self):
        self.arm_on()
        self.frame(grip=1)
        self.assertEqual(self.arm.calls, [])
        self.frame()
        self.frame(buttons={'y'})
        self.frame()
        self.assertTrue(self.c.gripper_ready)
        self.arm.calls.clear()
        self.frame(grip=1)
        self.assertEqual(self.arm.gripper, 420)
        self.assertEqual(self.moves(), [])
        count = len(self.arm.calls)
        self.frame()
        self.assertEqual(len(self.arm.calls), count)
        self.frame(xyz=(1, 0, 0), grip=1)
        self.assertEqual(self.arm.gripper, 420)
        self.assertEqual(self.moves(), [])

    def test_release_during_gripper_read_cancels_next_step(self):
        self.arm_on()
        self.frame(buttons={'y'})
        self.frame()
        self.arm.calls.clear()
        original = self.arm.get_gripper_position
        def release():
            self.c.publish_pad(replace(self.c.pad, xyz=(0, 0, 0), rxyz=(0, 0, 0), grip=0, buttons=frozenset()))
            return original()
        self.arm.get_gripper_position = release
        self.frame(grip=1)
        self.assertFalse(any(c[0] == 'gripper_position' for c in self.arm.calls))

    def test_native_gripper_setup_then_x_and_b_without_a(self):
        self.arm_on()
        def native(buttons):
            pad = x.native_pad(dict(version=2, connected=True, axes=[0]*6,
                                    buttons=buttons), self.clock())
            self.controls.frame(self.c, replace(pad, focused=True))
            self.c.tick()
        native(['y'])
        self.assertTrue(self.c.gripper_ready)
        self.assertFalse(self.c.neutral_required)
        self.assertFalse(any(c[0] == 'gripper_position' for c in self.arm.calls))
        native(['x'])
        self.assertEqual(self.arm.gripper, 420)
        native([])
        native(['b'])
        self.assertEqual(self.arm.gripper, 400)
        self.assertEqual(self.moves(), [])
        self.assertTrue(self.c.armed)

    def test_gripper_setup_rejects_every_displaced_motion_control(self):
        self.arm_on()
        for fields in [dict(xyz=(.2, 0, 0)), dict(rxyz=(0, .2, 0)),
                       dict(grip=1), dict(grip=-1), dict(buttons={'y', 'lb'}),
                       dict(buttons={'y', 'rt'})]:
            with self.subTest(fields=fields):
                self.frame()
                self.arm.calls.clear()
                self.frame(**(dict(buttons={'y'}) | fields))
                self.assertFalse(self.c.gripper_ready)
                self.assertFalse(any(c[0].startswith('gripper_') for c in self.arm.calls))
                self.assertIn("Center sticks", self.c.notice)

    def test_gripper_setup_moved_input_must_be_released_before_moving(self):
        self.arm_on()
        original = self.arm.set_gripper_speed
        def moved(speed):
            self.c.publish_pad(replace(self.c.pad, grip=1, buttons=frozenset({'x'})))
            return original(speed)
        self.arm.set_gripper_speed = moved
        self.frame(buttons={'y'})
        self.assertTrue(self.c.gripper_ready)
        self.assertTrue(self.c.neutral_required)
        self.frame(grip=1)
        self.assertFalse(any(c[0] == 'gripper_position' for c in self.arm.calls))
        self.frame()
        self.frame(grip=1)
        self.assertEqual(self.arm.gripper, 420)

    def test_faster_gripper_setup_does_not_move_or_change_force(self):
        self.arm_on()
        self.frame(buttons={'y'})
        self.assertEqual(self.arm.calls, [
            ('gripper_mode', 0), ('gripper_enable', True), ('gripper_speed', 1000)])
        self.assertEqual(self.arm.gripper, 400)
        self.assertTrue(self.c.gripper_ready)

    def test_faster_gripper_waits_for_feedback_without_accumulating_targets(self):
        self.arm_on()
        self.frame(buttons={'y'})
        self.frame()
        def delayed_position(position, **kw):
            self.arm.calls.append(('gripper_position', position, kw))
            return 0
        self.arm.set_gripper_position = delayed_position
        self.frame(grip=1)
        self.arm.gripper_status = 1
        for _ in range(10):
            self.clock.advance()
            self.frame(grip=1)
        moves = [call for call in self.arm.calls if call[0] == 'gripper_position']
        self.assertEqual(len(moves), 1)
        self.assertEqual(moves[0][1], 420)
        self.assertFalse(moves[0][2]['wait_motion'])
        self.frame()
        self.arm.gripper, self.arm.gripper_status = 420, 0
        self.clock.advance()
        self.frame()
        self.assertEqual(len([call for call in self.arm.calls if call[0] == 'gripper_position']), 1)
        self.frame(grip=-1)
        self.assertEqual(self.c.gripper_target, 400)

    def test_faster_gripper_clamps_both_travel_limits(self):
        for origin, direction, target in [(842, 1, 850), (8, -1, 0)]:
            self.setUp()
            self.arm_on()
            self.arm.gripper = origin
            self.frame(buttons={'y'})
            self.frame()
            self.frame(grip=direction)
            self.assertEqual(self.arm.gripper, target)
            count = len([call for call in self.arm.calls if call[0] == 'gripper_position'])
            self.frame(grip=direction)
            self.assertEqual(len([call for call in self.arm.calls if call[0] == 'gripper_position']), count)

    def test_stop_during_enable_and_during_move_is_last_write(self):
        original = self.arm.motion_enable
        def stop_enable(**kw):
            self.c.stop()
            return original(**kw)
        self.arm.motion_enable = stop_enable
        self.frame(buttons={'menu'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.assertNotIn(('mode', 7), self.arm.calls)
        self.setUp()
        self.arm_on()
        original_move = self.arm.set_position
        def stop_move(*args, **kw):
            self.c.stop()
            return original_move(*args, **kw)
        self.arm.set_position = stop_move
        self.frame(xyz=(1, 0, 0))
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_worker_close_stops_owned_arm(self):
        self.arm_on()
        self.c.start()
        self.c.close()
        self.assertFalse(self.c.worker.is_alive())
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_invalid_axes_and_acceleration_fail_closed(self):
        self.arm_on()
        self.frame(xyz=(float('nan'), 0, 0))
        self.assertFalse(self.c.armed)
        self.assertEqual(self.moves(), [])
        self.setUp()
        self.arm.tcp_acc_limit = [1, float('nan')]
        self.frame(buttons={'menu'})
        self.assertEqual(self.arm.calls, [])

    def test_deadzone_response_is_continuous_and_monotonic(self):
        self.assertEqual(x.shaped(0.14), 0)
        self.assertAlmostEqual(x.shaped(1), 1)
        self.assertAlmostEqual(x.shaped(-1), -1)
        self.assertLess(x.shaped(0.151), 0.001)
        values = [x.shaped(i/100) for i in range(101)]
        self.assertEqual(values, sorted(values))

    def test_input_only_never_prepares_or_connects_robot(self):
        with mock.patch.object(x, 'prepare_arm', side_effect=AssertionError('Robot access')), \
                mock.patch.object(x, 'window') as window:
            self.assertEqual(x.main(['--input-only']), 0)
        window.assert_called_once_with(input_only=True)

    def test_native_mapping_and_opposing_triggers(self):
        packet = {'version': 2, 'connected': True, 'name': 'Xbox', 'axes': [-1, 1, -1, 1, 1, 0],
                  'buttons': ['menu', 'x', 'a', 'lb']}
        pad = x.native_pad(packet, self.clock())
        self.assertEqual(pad.xyz, (1, 1, -1))
        self.assertEqual(pad.rxyz, (-1, 1, -1))
        self.assertEqual(pad.grip, 1)
        self.assertIn('a', pad.buttons)
        packet['axes'][5] = 1
        self.assertEqual(x.native_pad(packet, self.clock()).rxyz[2], 0)
        packet['axes'][4] = 0
        self.assertEqual(x.native_pad(packet, self.clock()).rxyz[2], 1)
        packet['buttons'] = ['rb', 'b', 'l3', 'r3']
        pad = x.native_pad(packet, self.clock())
        self.assertEqual(pad.xyz[2], 1)
        self.assertEqual(pad.grip, -1)
        self.assertIn('l3', pad.buttons)
        self.assertTrue(pad.precision)

    def test_invalid_native_packets_are_rejected(self):
        packet = {'version': 2, 'connected': True, 'name': 'Xbox', 'axes': [0]*6, 'buttons': []}
        for key, value in [('axes', [float('nan')]*6), ('axes', [True]*6),
                           ('axes', [0, 0, 0, 0, -1, 0]), ('version', 1),
                           ('buttons', ['unknown']), ('connected', 1)]:
            with self.subTest(key=key, value=value):
                with self.assertRaises(ValueError):
                    x.native_pad(dict(packet, **{key: value}), self.clock())

    def test_opposing_motion_buttons_prevent_enable(self):
        for buttons, axes in [(['lb', 'rb'], [0]*6), (['x', 'b'], [0]*6),
                              ([], [0, 0, 0, 0, 1, 1])]:
            pad = x.native_pad(dict(version=2, connected=True, buttons=buttons, axes=axes), 10)
            self.assertFalse(pad.neutral)
            self.assertEqual(x.demand(pad), ((0,)*6, 0))

    def test_b_closes_gripper_and_never_disarms(self):
        self.arm_on()
        self.frame(buttons={'y'})
        self.frame()
        pad = x.native_pad(dict(version=2, connected=True, axes=[0]*6, buttons=['b']), 10)
        self.controls.frame(self.c, replace(pad, focused=True))
        self.c.tick()
        self.assertTrue(self.c.armed)
        self.assertEqual(self.arm.gripper, 380)
        self.assertEqual(self.moves(), [])

    def test_rotation_works_directly_and_is_bounded(self):
        self.arm_on()
        self.arm.pose[3:] = [0, 0, 0]
        for _ in range(10):
            self.clock.advance()
            self.frame(rxyz=(1, 1, 1))
        self.assertEqual(len(self.moves()), 10)
        for kind, target, options in self.moves():
            self.assertEqual(kind, 'rotation')
            self.assertEqual(target[:3], self.arm.pose[:3])
            self.assertAlmostEqual(math.sqrt(sum(v*v for v in target[3:])),
                                   math.radians(x.LOOKAHEAD_DEGREES))
            self.assertAlmostEqual(options['speed'], 50)
            self.assertEqual(options['mvacc'], 1000)
            self.assertFalse(options['wait'])
            self.assertTrue(options['is_radian'])
            self.assertFalse(options['is_tool_coord'])
            self.assertFalse(options['relative'])
        self.frame()
        self.assertEqual(self.arm.calls[-1], ('state', 6))

    def test_rotation_is_about_base_axis_not_tool_or_euler_axis(self):
        # Starting at 180 deg roll, +base-Y rotation moves the tool's Z toward -X.
        pose = [300, 0, 400, 180, 0, 0]
        target = x.base_rotation_target(pose, (0, 1, 0))
        rv = target[3:]
        angle = math.sqrt(sum(v*v for v in rv))
        u = [v/angle for v in rv]
        # Rodrigues formula, applied to the local Z basis vector.
        z = [u[1]*math.sin(angle) + u[0]*u[2]*(1-math.cos(angle)),
             -u[0]*math.sin(angle) + u[1]*u[2]*(1-math.cos(angle)),
             math.cos(angle) + u[2]**2*(1-math.cos(angle))]
        self.assertAlmostEqual(z[0], -math.sin(math.radians(8)))
        self.assertAlmostEqual(z[1], 0)
        self.assertAlmostEqual(z[2], -math.cos(math.radians(8)))

    def test_translation_rotation_and_grip_conflicts_stop(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        before = len(self.moves())
        self.frame(xyz=(1, 0, 0), rxyz=(0, 1, 0))
        self.assertEqual(len(self.moves()), before)
        self.assertEqual(self.arm.calls[-1], ('state', 6))
        self.frame(rxyz=(0, 1, 0), grip=-1)
        self.assertEqual(len(self.moves()), before)

    def test_waits_for_deceleration_before_switching_motion_group(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.clock.advance()
        self.frame(rxyz=(0, 0, 1))
        self.assertEqual(self.arm.calls[-1], ('state', 6))
        self.arm.state = 6
        self.clock.advance()
        self.frame(rxyz=(0, 0, 1))
        self.assertEqual(len(self.moves()), 1)
        self.arm.state = 2
        self.frame()  # Center the new motion controls before resuming.
        self.clock.advance()
        self.frame(rxyz=(0, 0, 1))
        self.assertEqual(self.moves()[-1][0], 'rotation')

    def test_repeated_move_release_move_does_not_require_menu(self):
        for stopped_state in (4, 5):
            with self.subTest(stopped_state=stopped_state):
                self.setUp()
                self.arm.soft_stop_state = stopped_state
                self.arm_on()
                for i in range(4):
                    self.clock.advance()
                    self.frame(xyz=(1, 0, 0))
                    self.frame()  # Centering alone stops each move.
                    self.assertEqual(self.arm.state, stopped_state)
                    self.assertTrue(self.c.armed, self.c.message)
                self.assertEqual(len(self.moves()), 4)
                self.assertEqual(self.arm.calls.count(('state', 0)), 3)
                self.assertFalse(any(c[0] in ('enable', 'mode') for c in self.arm.calls))

    def test_changed_configuration_during_release_stop_never_resumes(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.frame()
        self.arm.tcp_load = [5, [1, 2, 3]]
        self.clock.advance()
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertNotIn(('state', 0), self.arm.calls)
        self.assertEqual(len(self.moves()), 1)

    def test_resume_waits_for_configuration_report_newer_than_stop(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.frame()
        reports = mock.Mock()
        reports.updated = self.clock() - 0.01
        self.c.reports = reports
        self.clock.advance()
        with mock.patch.object(x.time, 'monotonic', side_effect=self.clock):
            self.frame(xyz=(1, 0, 0))
        self.assertTrue(self.c.armed)
        self.assertNotIn(('state', 0), self.arm.calls)
        reports.updated = self.clock()
        with mock.patch.object(x.time, 'monotonic', side_effect=self.clock):
            self.frame(xyz=(1, 0, 0))
        self.assertIn(('state', 0), self.arm.calls)

    def test_unexpected_stop_is_never_automatically_cleared(self):
        self.arm_on()
        self.arm.state = 4
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertNotIn(('state', 0), self.arm.calls)
        self.assertEqual(self.moves(), [])

    def test_a_l3_have_no_action_and_releasing_them_does_not_stop_stick_motion(self):
        self.arm_on()
        for buttons in ({'a'}, {'l3'}, {'a', 'l3'}, set()):
            self.frame(buttons=buttons)
            self.assertEqual(self.arm.calls, [])
        for buttons in ({'a'}, {'l3'}, set()):
            self.clock.advance()
            self.frame(xyz=(1, 0, 0), buttons=buttons)
        self.assertEqual(len(self.moves()), 3)
        self.assertNotIn(('state', 6), self.arm.calls)
        self.frame()
        self.assertEqual(self.arm.calls[-1], ('state', 6))

    def test_live_fault_read_blocks_resume_even_when_cached_status_is_clear(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.frame()
        self.arm.get_err_warn_code = lambda: (0, [19, 0])
        self.clock.advance()
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertNotIn(('state', 0), self.arm.calls)
        self.assertEqual(len(self.moves()), 1)

    def test_stop_button_or_focus_loss_cancels_release_resume_permission(self):
        for kwargs in ({'buttons': {'view'}}, {'focused': False}):
            self.setUp()
            self.arm_on()
            self.frame(xyz=(1, 0, 0))
            self.frame()
            self.frame(**kwargs)
            self.assertIsNone(self.c.release_stop)
            self.clock.advance()
            self.frame(xyz=(1, 0, 0))
            self.assertFalse(self.c.armed)
            self.assertNotIn(('state', 0), self.arm.calls)

    def test_input_release_during_resume_checks_does_not_restore_standby(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.frame()
        original = self.arm.get_err_warn_code
        def release():
            self.c.publish_pad(replace(self.c.pad, xyz=(0, 0, 0), rxyz=(0, 0, 0), grip=0, buttons=frozenset()))
            return original()
        self.arm.get_err_warn_code = release
        self.clock.advance()
        self.frame(xyz=(1, 0, 0))
        self.assertNotIn(('state', 0), self.arm.calls)
        self.assertEqual(len(self.moves()), 1)

    def test_unexpected_queue_blocks_resume(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.frame()
        self.arm.queue = 1
        self.clock.advance()
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertNotIn(('state', 0), self.arm.calls)

    def test_deceleration_timeout_disarms_without_resuming(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.frame()
        self.arm.state = 6
        self.clock.advance(2.1)
        self.frame()
        self.assertFalse(self.c.armed)
        self.assertNotIn(('state', 0), self.arm.calls)

    def test_analog_update_during_target_calculation_does_not_stop(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.clock.advance()
        health = self.c.health
        active_checks = 0
        def changing_input(active=False):
            nonlocal active_checks
            result = health(active)
            active_checks += int(active)
            if active_checks == 2:
                self.c.publish_pad(replace(self.c.pad, xyz=(0.6, 0, 0)))
            return result
        self.c.health = changing_input
        self.frame(xyz=(0.5, 0, 0))
        self.assertNotIn(('state', 6), self.arm.calls)
        self.assertTrue(self.c.armed)
        self.c.health = health
        self.frame(xyz=(0.6, 0, 0))
        self.assertEqual(len(self.moves()), 2)

    def test_resume_waits_for_new_fast_ready_report(self):
        self.arm_on()
        feedback = mock.Mock()
        feedback.check.return_value = x.Feedback(tuple(self.arm.pose), 7, 2, self.clock())
        self.c.feedback = feedback
        self.frame(xyz=(1, 0, 0))
        self.frame()
        self.clock.advance()
        feedback.check.return_value = x.Feedback(tuple(self.arm.pose), 7, 4, self.clock())
        self.frame(xyz=(1, 0, 0))
        self.assertIn(('state', 0), self.arm.calls)
        self.assertEqual(len(self.moves()), 1)
        self.assertTrue(self.c.armed, self.c.message)
        self.clock.advance()
        feedback.check.return_value = x.Feedback(tuple(self.arm.pose), 7, 2, self.clock())
        self.frame(xyz=(1, 0, 0))
        self.assertEqual(len(self.moves()), 2)
        self.assertIsNone(self.c.release_stop)

    def test_stop_during_standby_restore_is_last_write(self):
        self.arm_on()
        self.frame(xyz=(1, 0, 0))
        self.frame()
        original = self.arm.set_state
        def stopped_during_resume(state):
            if state == 0:
                self.c.stop("Operator stop during resume")
            return original(state)
        self.arm.set_state = stopped_during_resume
        self.clock.advance()
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertIsNone(self.c.release_stop)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.assertEqual(len(self.moves()), 1)

    def test_resume_confirmation_timeout_never_sends_another_target(self):
        self.arm_on()
        feedback = mock.Mock()
        feedback.check.return_value = x.Feedback(tuple(self.arm.pose), 7, 2, self.clock())
        self.c.feedback = feedback
        self.frame(xyz=(1, 0, 0))
        self.frame()
        self.clock.advance()
        feedback.check.return_value = x.Feedback(tuple(self.arm.pose), 7, 4, self.clock())
        self.frame(xyz=(1, 0, 0))
        self.clock.advance(0.6)
        feedback.check.return_value = x.Feedback(tuple(self.arm.pose), 7, 4, self.clock())
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertEqual(len(self.moves()), 1)
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_rich_200ms_cadence_passes_but_500ms_outage_stops(self):
        self.arm_on()
        reports = ReportWatch()
        reports.received.set()
        reports.updated = 10
        self.c.reports = reports
        with mock.patch.object(x.time, 'monotonic', return_value=10.21):
            self.frame(xyz=(1, 0, 0))
        self.assertTrue(self.c.armed, self.c.message)
        with mock.patch.object(x.time, 'monotonic', return_value=10.51):
            self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_fast_feedback_staleness_stops_and_requires_explicit_reenable(self):
        self.arm_on()
        feedback = object.__new__(x.FastFeedback)
        feedback.latest = x.Feedback((500, 0, 400, 180, 0, 0), 7, 2, 10)
        feedback.closed = False
        feedback.error = None
        self.c.feedback = feedback
        with mock.patch.object(x.time, 'monotonic', return_value=10.01):
            self.frame(xyz=(1, 0, 0))
        self.assertEqual(self.moves()[-1][1][0], 560)  # Uses fast pose, not rich pose=300.
        with mock.patch.object(x.time, 'monotonic', return_value=10.16):
            self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        count = len(self.moves())
        feedback.latest = replace(feedback.latest, updated=10.17)
        with mock.patch.object(x.time, 'monotonic', return_value=10.18):
            self.frame(xyz=(1, 0, 0))
        self.assertEqual(len(self.moves()), count)

    def test_fast_mode_change_prevents_motion_before_rich_report(self):
        self.arm_on()
        self.c.feedback = mock.Mock()
        self.c.feedback.check.return_value = x.Feedback(tuple(self.arm.pose), 0, 2, 10)
        self.frame(xyz=(1, 0, 0))
        self.assertFalse(self.c.armed)
        self.assertEqual(self.moves(), [])

    def test_enable_waits_for_fresh_fast_standby_without_moving(self):
        self.c.feedback = mock.Mock()
        self.c.feedback.check.side_effect = lambda: x.Feedback(
            tuple(self.arm.pose), self.arm.mode, self.arm.state, x.time.monotonic())
        self.arm_on()
        self.assertTrue(self.c.armed)
        self.assertEqual(self.moves(), [])

    def test_enable_waits_for_delayed_rich_mode_then_direct_input_works(self):
        self.c.reports = mock.Mock(updated=self.clock())
        self.c.feedback = mock.Mock()
        # The controller changes mode immediately, but the SDK cache lags.
        self.arm.set_mode = lambda mode: self.arm.calls.append(('mode', mode)) or 0
        self.c.feedback.check.side_effect = lambda: x.Feedback(
            tuple(self.arm.pose), 7 if ('mode', 7) in self.arm.calls else 0,
            self.arm.state, self.clock())
        start = self.clock()
        def advance(seconds):
            self.clock.advance(seconds)
            self.c.publish_pad(replace(self.c.pad, updated=self.clock()))
            if self.clock() - start >= .2:
                self.arm.mode = 7
                self.c.reports.updated = self.clock()
            self.assertFalse(self.c.armed)
            self.assertEqual(self.moves(), [])
        with mock.patch.object(x.time, 'monotonic', side_effect=self.clock), \
                mock.patch.object(x.time, 'sleep', side_effect=advance):
            self.frame(buttons={'menu'})
            self.assertTrue(self.c.armed, self.c.message)
            self.assertGreaterEqual(self.clock() - start, .2)
            self.frame()
            self.frame(xyz=(1, 0, 0))
            self.assertTrue(self.c.armed, self.c.message)
            self.assertEqual(len(self.moves()), 1)
            self.assertNotIn('a', self.c.pad.buttons)

    def test_enable_requires_fresh_rich_report_and_correct_cached_mode(self):
        for refresh, correct_mode in [(False, True), (True, False)]:
            with self.subTest(refresh=refresh, correct_mode=correct_mode):
                self.setUp()
                self.c.reports = mock.Mock(updated=self.clock())
                self.c.feedback = mock.Mock()
                self.arm.set_mode = lambda mode: self.arm.calls.append(('mode', mode)) or 0
                self.c.feedback.check.side_effect = lambda: x.Feedback(
                    tuple(self.arm.pose), 7 if ('mode', 7) in self.arm.calls else 0,
                    self.arm.state, self.clock())
                def advance(seconds):
                    self.clock.advance(seconds)
                    self.c.publish_pad(replace(self.c.pad, updated=self.clock()))
                    if refresh:
                        self.c.reports.updated = self.clock()
                    if correct_mode:
                        self.arm.mode = 7
                with mock.patch.object(x.time, 'monotonic', side_effect=self.clock), \
                        mock.patch.object(x.time, 'sleep', side_effect=advance):
                    self.frame(buttons={'menu'})
                self.assertFalse(self.c.armed)
                self.assertIn("both feedback streams", self.c.message)
                self.assertEqual(self.moves(), [])
                self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_fault_or_operator_stop_during_enable_confirmation_stops(self):
        for fault in (True, False):
            with self.subTest(fault=fault):
                self.setUp()
                self.c.reports = mock.Mock(updated=self.clock())
                def advance(seconds):
                    self.clock.advance(seconds)
                    self.c.publish_pad(replace(self.c.pad, updated=self.clock()))
                    if fault:
                        self.arm.error = 24
                    else:
                        self.c.stop("Operator stop during enable confirmation")
                with mock.patch.object(x.time, 'monotonic', side_effect=self.clock), \
                        mock.patch.object(x.time, 'sleep', side_effect=advance):
                    self.frame(buttons={'menu'})
                self.assertFalse(self.c.armed)
                self.assertEqual(self.moves(), [])
                self.assertEqual(self.arm.calls[-1], ('state', 4))
                if fault:
                    self.assertEqual(self.arm.error, 24)

    def test_no_enable_writes_with_failed_fast_feedback(self):
        self.c.feedback = mock.Mock()
        self.c.feedback.check.side_effect = RuntimeError('Real-time feedback is stale')
        self.frame(buttons={'menu'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls, [])

    def test_rotation_speed_scales_with_reduced_limit(self):
        self.arm.is_reduced_mode = True
        self.arm.reduced_max_tcp_speed = 115
        self.arm_on()
        self.frame(rxyz=(0, 0, 1))
        self.assertAlmostEqual(self.moves()[-1][2]['speed'], 25)

    def test_native_rotations_all_reach_planner_and_respect_precision(self):
        self.arm_on()
        self.arm.pose[3:] = [0, 0, 0]
        for axes, expected in [([0, 0, 1, 0, 0, 0], (1, 0, 0)),
                               ([0, 0, 0, 1, 0, 0], (0, 1, 0)),
                               ([0, 0, 0, 0, 1, 0], (0, 0, -1)),
                               ([0, 0, 0, 0, 0, 1], (0, 0, 1))]:
            self.clock.advance()
            pad = x.native_pad(dict(version=2, connected=True, axes=axes,
                                    buttons=['r3']), self.clock())
            self.controls.frame(self.c, replace(pad, focused=True))
            self.c.tick()
            kind, target, options = self.moves()[-1]
            self.assertEqual(kind, 'rotation')
            self.assertEqual(target[:3], self.arm.pose[:3])
            for actual, unit in zip(target[3:], expected):
                self.assertAlmostEqual(actual, unit * math.radians(8))
            self.assertEqual(options['speed'], 10)
        self.frame()
        self.assertEqual(self.arm.calls[-1], ('state', 6))

    def test_rotation_uses_lower_controller_acceleration_limit(self):
        self.arm.tcp_acc_limit = [1, 300]
        self.arm_on()
        self.frame(rxyz=(1, 0, 0))
        self.assertEqual(self.moves()[-1][2]['mvacc'], 300)

    def test_feedback_parser_units_and_malformed_packets(self):
        packet = bytearray(135)
        packet[:4] = (135).to_bytes(4, 'big')
        packet[4] = (7 << 4) | 2
        struct.pack_into('<6f', packet, 35, 300, -100, 400, math.pi, 0, 0)
        result = x.parse_feedback(packet, 10)
        self.assertEqual(result.mode, 7)
        self.assertEqual(result.state, 2)
        self.assertEqual(result.pose[:3], (300, -100, 400))
        self.assertAlmostEqual(result.pose[3], 180, places=4)
        with self.assertRaises(ValueError):
            x.parse_feedback(packet[:-1], 10)
        struct.pack_into('<f', packet, 35, float('nan'))
        with self.assertRaises(ValueError):
            x.parse_feedback(packet, 10)

    def test_fast_reader_fragmented_frames_and_disconnect(self):
        packet = bytearray(135)
        packet[:4] = (135).to_bytes(4, 'big')
        packet[4] = (7 << 4) | 2
        feedback = object.__new__(x.FastFeedback)
        feedback.closed = False
        feedback.latest = feedback.error = None
        feedback.received = threading.Event()
        feedback.socket = mock.Mock()
        feedback.socket.recv.side_effect = [packet[:2], packet[2:50], packet[50:] + packet, b'']
        feedback.consume()
        self.assertEqual(feedback.latest.mode, 7)
        self.assertTrue(feedback.received.is_set())
        with self.assertRaisesRegex(RuntimeError, 'disconnected'):
            feedback.check()
        feedback.socket.send.assert_not_called()
        feedback.socket.sendall.assert_not_called()

    def test_native_reader_freeze_and_exit_fail_closed(self):
        reader = object.__new__(x.NativePadReader)
        reader.process = mock.Mock()
        reader.process.poll.return_value = None
        reader.latest = x.Pad(True, xyz=(1, 0, 0), updated=10)
        with mock.patch.object(x.time, 'monotonic', return_value=10.05):
            self.assertTrue(reader.read(True).connected)
        with mock.patch.object(x.time, 'monotonic', return_value=10.2):
            self.assertFalse(reader.read(True).connected)
        reader.process.poll.return_value = 1
        with mock.patch.object(x.time, 'monotonic', return_value=10.05):
            self.assertFalse(reader.read(True).connected)

    def test_simultaneous_local_control_windows_are_rejected(self):
        lock = x.acquire_control_lock()
        try:
            with self.assertRaises(RuntimeError):
                x.acquire_control_lock()
        finally:
            lock.close()


if __name__ == '__main__':
    unittest.main()
