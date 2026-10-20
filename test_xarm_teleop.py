"""Offline tests: no real robot connections or motion commands."""
import contextlib
import io
import math
import unittest
from unittest import mock
import xarm_teleop as t


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def advance(self, seconds=0.06):
        self.now += seconds


class FakeArm:
    connected = True
    axis = 6
    mode = 0
    version = 'offline fake'
    version_number = (2, 7, 1)
    tcp_load = [4, [0, 0, 0]]
    tcp_offset = [0] * 6
    tcp_speed_limit = [0.1, 1000.0]
    is_reduced_mode = False
    reduced_max_tcp_speed = 1000.0

    def __init__(self):
        self.state = 4
        self.queue = 0
        self.error = self.warning = 0
        self.pose = [300, 0, 400, 180, 0, 0]
        self.angles = [0] * 7
        self.gripper = 400.0
        self.gripper_error = 0
        self.gripper_status = 0
        self.calls = []

    def get_err_warn_code(self): return 0, [self.error, self.warning]
    def get_state(self): return 0, self.state
    def get_cmdnum(self): return 0, self.queue
    def get_position(self, **kw): return 0, self.pose[:]
    def get_servo_angle(self, **kw): return 0, self.angles[:]
    def get_gripper_err_code(self): return 0, self.gripper_error
    def get_gripper_position(self): return 0, self.gripper
    def get_gripper_status(self): return 0, self.gripper_status
    def get_gripper_version(self): return 0, '5.2.1'
    def get_reduced_states(self, **kw):
        return 0, [int(self.is_reduced_mode), [], self.reduced_max_tcp_speed, 180, [], 0, 1]

    def set_state(self, state):
        self.calls.append(('state', state))
        self.state = state
        return 0

    def motion_enable(self, **kw):
        self.calls.append(('enable', kw))
        return 0

    def set_mode(self, mode):
        self.calls.append(('mode', mode))
        self.mode = mode
        return 0

    def vc_set_cartesian_velocity(self, speeds, **kw):
        self.calls.append(('velocity', tuple(speeds), kw))
        return 0

    def set_gripper_mode(self, mode):
        self.calls.append(('gripper_mode', mode))
        return 0

    def set_gripper_enable(self, enable):
        self.calls.append(('gripper_enable', enable))
        return 0

    def set_gripper_speed(self, speed):
        self.calls.append(('gripper_speed', speed))
        return 0

    def set_gripper_position(self, position, **kw):
        self.calls.append(('gripper_position', position, kw))
        self.gripper = position
        return 0


class HoldTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.arm = FakeArm()
        self.c = t.Teleop(self.arm, clock=self.clock)
        self.k = t.Keyboard()

    def frame(self, held=(), focused=True, **kw):
        result = self.k.frame(self.c, held, focused, **kw)
        self.c.tick()
        return result

    def arm_on(self):
        self.frame({'e'})
        self.frame()
        self.assertTrue(self.c.armed)
        self.arm.calls.clear()

    def grip_on(self):
        self.arm_on()
        self.frame({'g'})
        self.frame()
        self.assertTrue(self.c.gripper_ready)
        self.arm.calls.clear()

    def velocities(self):
        return [call for call in self.arm.calls if call[0] == 'velocity']

    def gripper_moves(self):
        return [call for call in self.arm.calls if call[0] == 'gripper_position']

    def test_exact_key_mapping(self):
        expected = {'w': (1000, 0, 0), 's': (-1000, 0, 0),
                    'a': (0, 1000, 0), 'd': (0, -1000, 0),
                    'up': (0, 0, 1000), 'down': (0, 0, -1000)}
        for key, xyz in expected.items():
            with self.subTest(key=key):
                speed, grip = t.intent({key})
                self.assertEqual(speed, xyz + (0, 0, 0))
                self.assertEqual(grip, 0)
        self.assertEqual(t.intent({'left'}), (t.ZERO, 1))
        self.assertEqual(t.intent({'right'}), (t.ZERO, -1))
        self.assertEqual(t.intent({'r', 'f', 'o', 'c'}), (t.ZERO, 0))

    def test_opposing_keys_cancel_and_diagonal_speed_is_capped(self):
        self.assertEqual(t.intent(t.MOVE_KEYS), (t.ZERO, 0))
        speed, _ = t.intent({'w', 'a', 'up'})
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in speed)), 1000)
        self.assertEqual(speed[3:], (0, 0, 0))

    def test_startup_and_disarmed_keys_never_write(self):
        self.frame(t.MOVE_KEYS)
        self.c.close()
        self.assertEqual(self.arm.calls, [])

    def test_single_e_press_enables_without_enter_but_does_not_move(self):
        self.frame({'e'})
        self.assertTrue(self.c.armed)
        self.assertIn(('mode', 5), self.arm.calls)
        self.assertEqual(self.velocities()[0][1], t.ZERO)
        original = len(self.arm.calls)
        self.frame({'e'})
        self.assertEqual(len(self.arm.calls), original)

    def test_hold_renews_velocity_without_repeat_events_or_enter(self):
        self.arm_on()
        for _ in range(20):
            self.clock.advance()
            self.frame({'w'})
        commands = self.velocities()
        self.assertEqual(len(commands), 20)
        for _, speed, options in commands:
            self.assertEqual(speed, (1000, 0, 0, 0, 0, 0))
            self.assertEqual(options['duration'], 0.1)
            self.assertFalse(options['is_tool_coord'])

    def test_release_sends_zero_and_remains_armed(self):
        self.arm_on()
        self.frame({'up'})
        self.frame()
        self.assertEqual(self.velocities()[-1][1], t.ZERO)
        self.assertTrue(self.c.armed)
        self.frame({'down'})
        self.assertEqual(self.velocities()[-1][1], (0, 0, -1000, 0, 0, 0))

    def test_no_motion_until_keys_released_after_enable(self):
        self.c.publish((), True)
        self.assertTrue(self.c.request('enable'))
        self.c.publish({'w'}, True)
        self.c.tick()
        self.c.tick()
        self.assertTrue(self.c.armed)
        self.assertTrue(all(call[1] == t.ZERO for call in self.velocities()))
        self.c.publish((), True)
        self.c.tick()
        self.c.publish({'w'}, True)
        self.c.tick()
        self.assertEqual(self.velocities()[-1][1][0], 1000)

    def test_enable_with_movement_held_is_rejected(self):
        self.frame({'e', 'w'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls, [])

    def test_space_wins_over_enable_and_motion(self):
        self.frame({'e', 'w', 'space'})
        self.assertEqual(self.arm.calls, [])
        self.frame()  # A fresh E press, after releasing the stopped key chord.
        self.arm_on()
        self.frame({'w'})
        self.frame({'w', 'space', 'e'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.frame({'w'})
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_focus_loss_disarms_and_focus_return_does_not_resume(self):
        self.arm_on()
        self.frame({'w'})
        self.frame({'w'}, focused=False)
        self.assertFalse(self.c.armed)
        count = len(self.arm.calls)
        self.frame({'w'}, focused=True)
        self.assertEqual(len(self.arm.calls), count)

    def test_transient_focus_loss_in_same_frame_still_disarms(self):
        self.arm_on()
        self.frame({'w'}, focused=True, focus_lost=True)
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_ui_heartbeat_expiration_stops_motion(self):
        self.arm_on()
        self.frame({'w'})
        self.clock.advance(0.3)
        self.c.tick()
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_expired_input_cannot_enable(self):
        self.c.publish((), True)
        self.clock.advance(0.3)
        self.assertFalse(self.c.request('enable'))
        self.assertEqual(self.arm.calls, [])

    def test_early_stop_cancels_pending_enable_and_allows_fresh_enable(self):
        self.c.publish((), True)
        self.c.request('enable')
        self.c.stop()
        self.c.tick()
        self.assertFalse(self.c.busy)
        self.assertEqual(self.arm.calls, [])
        self.assertTrue(self.c.request('enable'))
        self.c.tick()
        self.assertTrue(self.c.armed)

    def test_stop_during_enable_preflight_cannot_be_overwritten(self):
        original = self.arm.get_cmdnum
        def stop_during_read():
            self.c.stop()
            return original()
        self.arm.get_cmdnum = stop_during_read
        self.frame({'e'})
        self.assertEqual(self.arm.calls, [])
        self.assertFalse(self.c.armed)

    def test_stop_during_enable_write_prevents_following_enable_commands(self):
        original = self.arm.motion_enable
        def stop_during_write(**kw):
            self.c.stop()
            return original(**kw)
        self.arm.motion_enable = stop_during_write
        self.frame({'e'})
        self.assertNotIn(('mode', 5), self.arm.calls)
        self.assertNotIn(('state', 0), self.arm.calls)
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_release_during_health_read_uses_fresh_input(self):
        self.arm_on()
        self.frame({'w'})
        original = self.arm.get_state
        def release_during_read():
            self.c.publish((), True)
            return original()
        self.arm.get_state = release_during_read
        self.frame({'w'})
        self.assertEqual(self.velocities()[-1][1], t.ZERO)

    def test_stop_during_inflight_velocity_is_last_write(self):
        self.arm_on()
        original = self.arm.vc_set_cartesian_velocity
        def stop_during_write(*args, **kw):
            self.c.stop()
            return original(*args, **kw)
        self.arm.vc_set_cartesian_velocity = stop_during_write
        self.frame({'w'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.frame({'w'})
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_old_firmware_queued_motion_faults_and_motion_block_enable(self):
        for field, value in [('version_number', (1, 7, 9)), ('queue', 1),
                             ('error', 19), ('warning', 1), ('state', 1), ('axis', 7)]:
            with self.subTest(field=field):
                arm = FakeArm()
                setattr(arm, field, value)
                c = t.Teleop(arm, clock=self.clock)
                c.publish((), True)
                c.request('enable')
                c.tick()
                self.assertFalse(c.armed)
                self.assertEqual(arm.calls, [])

    def test_external_mode_change_disarms(self):
        self.arm_on()
        self.arm.mode = 0
        self.frame({'w'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.assertEqual(self.velocities(), [])

    def test_gripper_hold_open_close_and_release(self):
        self.grip_on()
        for _ in range(4):
            self.clock.advance()
            self.frame({'left'})
        self.assertEqual(self.arm.gripper, 420)
        count = len(self.gripper_moves())
        for _ in range(5):
            self.clock.advance()
            self.frame()
        self.assertEqual(len(self.gripper_moves()), count)
        self.frame({'right'})
        self.assertEqual(self.arm.gripper, 415)
        self.assertFalse(self.gripper_moves()[-1][2]['wait_motion'])

    def test_gripper_does_not_accumulate_targets_if_feedback_lags(self):
        self.grip_on()
        def delayed_position(pos, **kw):
            self.arm.calls.append(('gripper_position', pos, kw))
            return 0
        self.arm.set_gripper_position = delayed_position
        for _ in range(30):
            self.clock.advance()
            self.frame({'left'})
        self.assertEqual(len(self.gripper_moves()), 1)
        self.assertEqual(self.gripper_moves()[0][1], 405)
        self.frame()
        self.assertEqual(len(self.gripper_moves()), 1)

    def test_release_during_gripper_read_prevents_next_step(self):
        self.grip_on()
        original = self.arm.get_gripper_position
        def release_during_read():
            self.c.publish((), True)
            return original()
        self.arm.get_gripper_position = release_during_read
        self.frame({'left'})
        self.assertEqual(self.gripper_moves(), [])

    def test_gripper_end_limits_and_opposing_keys_do_not_move(self):
        self.grip_on()
        self.arm.gripper = 850
        self.frame({'left'})
        self.arm.gripper = 0
        self.frame({'right'})
        self.frame({'left', 'right'})
        self.assertEqual(self.gripper_moves(), [])

    def test_arm_and_gripper_directions_are_not_run_together(self):
        self.grip_on()
        self.frame({'w'})
        self.frame({'w', 'left'})
        self.assertEqual(self.velocities()[-1][1], t.ZERO)
        self.assertEqual(self.gripper_moves(), [])

    def test_stop_preserves_gripping_force(self):
        self.grip_on()
        self.frame({'right'})
        self.frame({'space'})
        self.assertNotIn(('gripper_enable', False), self.arm.calls)
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def stalled_gripper(self):
        def send(position, **kw):
            self.arm.calls.append(('gripper_position', position, kw))
            return 0
        self.arm.set_gripper_position = send

    def test_stopped_short_of_target_can_reverse_without_false_timeout(self):
        self.grip_on()
        self.arm.gripper = 813
        self.stalled_gripper()
        self.frame({'left'})
        self.clock.advance(4)
        self.frame({'right'})
        self.assertTrue(self.c.gripper_ready, self.c.message)
        self.assertEqual(self.gripper_moves()[-1][1], 808)

    def test_grasp_is_completion_and_does_not_keep_squeezing(self):
        self.grip_on()
        self.stalled_gripper()
        self.frame({'right'})
        self.arm.gripper_status = 2
        for _ in range(6):
            self.clock.advance(1)
            self.frame({'right'})
        self.assertTrue(self.c.gripper_ready, self.c.message)
        self.assertEqual(len(self.gripper_moves()), 1)
        self.frame({'left'})
        self.assertEqual(self.gripper_moves()[-1][1], 405)

    def test_full_speed_and_diagonals_obey_controller_limit(self):
        self.arm.tcp_speed_limit = [0.1, 600]
        self.arm_on()
        self.frame({'w', 'a'})
        self.assertAlmostEqual(math.sqrt(sum(v*v for v in self.velocities()[-1][1])), 600)

    def test_reduced_mode_change_limits_next_velocity(self):
        self.arm_on()
        self.frame({'w'})
        self.arm.is_reduced_mode = True
        self.arm.reduced_max_tcp_speed = 100
        self.frame({'w'})
        self.assertEqual(self.velocities()[-1][1], (100, 0, 0, 0, 0, 0))

    def test_reduced_mode_at_enable_and_invalid_limits_fail_closed(self):
        self.arm.is_reduced_mode = True
        self.arm.reduced_max_tcp_speed = 75
        self.arm_on()
        self.frame({'up'})
        self.assertEqual(self.velocities()[-1][1], (0, 0, 75, 0, 0, 0))
        self.arm.reduced_max_tcp_speed = float('nan')
        self.frame({'up'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        for limit in ([], [0, 0], [0, -5], [0, True], [0, float('nan')]):
            with self.subTest(limit=limit):
                arm = FakeArm()
                arm.tcp_speed_limit = limit
                c = t.Teleop(arm, clock=self.clock)
                c.publish((), True)
                c.request('enable')
                c.tick()
                self.assertFalse(c.armed)
                self.assertEqual(arm.calls, [])

    def test_stopped_short_with_progress_continues_held_movement(self):
        self.grip_on()
        self.stalled_gripper()
        self.frame({'left'})  # Requested 405.
        self.arm.gripper = 403  # Firmware says stopped, outside old one-pulse tolerance.
        self.frame({'left'})
        self.assertTrue(self.c.gripper_ready)
        self.assertEqual(self.gripper_moves()[-1][1], 408)

    def test_stopped_at_mechanical_limit_stays_enabled_without_repeated_commands(self):
        self.grip_on()
        self.arm.gripper = 813
        self.stalled_gripper()
        for _ in range(8):
            self.clock.advance(1)
            self.frame({'left'})
        self.assertTrue(self.c.gripper_ready, self.c.message)
        self.assertEqual(len(self.gripper_moves()), 1)
        self.assertIn('Reverse', self.c.message)

    def test_moving_status_prevents_queue_even_at_target_then_times_out(self):
        self.grip_on()
        self.frame({'left'})
        self.arm.gripper_status = 1
        for _ in range(5):
            self.clock.advance(0.5)
            self.frame({'left'})
        self.assertEqual(len(self.gripper_moves()), 1)
        self.assertTrue(self.c.armed)
        self.clock.advance(1)
        self.frame({'left'})
        self.assertFalse(self.c.armed)
        self.assertIn('still reports motion', self.c.message)

    def test_reenable_gripper_discards_completed_old_target(self):
        self.grip_on()
        self.stalled_gripper()
        self.frame({'left'})
        self.frame({'space'})
        self.clock.advance(5)
        self.frame()
        self.frame({'e'})
        self.frame()
        self.frame({'g'})
        self.frame()
        self.assertTrue(self.c.gripper_ready)
        self.frame({'right'})
        self.assertEqual(self.gripper_moves()[-1][1], 395)

    def test_gripper_fault_or_invalid_status_stops_without_more_steps(self):
        for field, value in [('gripper_error', 4), ('gripper_status', 3),
                             ('gripper_status', False), ('gripper_status', -1)]:
            with self.subTest(field=field, value=value):
                self.setUp()
                self.grip_on()
                setattr(self.arm, field, value)
                self.frame({'left'})
                self.assertFalse(self.c.armed)
                self.assertEqual(self.gripper_moves(), [])

    def test_moving_or_old_gripper_cannot_be_enabled(self):
        for moving in (False, True):
            with self.subTest(moving=moving):
                self.setUp()
                self.arm_on()
                if moving:
                    self.arm.gripper_status = 1
                else:
                    self.arm.get_gripper_version = lambda: (0, '3.4.2')
                self.frame({'g'})
                self.assertFalse(self.c.gripper_ready)
                self.assertFalse(any(c[0].startswith('gripper_') for c in self.arm.calls))

    def test_quit_stops_and_shuts_down_worker(self):
        self.arm_on()
        self.c.start()
        self.c.close()
        self.assertFalse(self.c.worker.is_alive())
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_status_reads_fault_without_writes(self):
        self.arm.error = 19
        with contextlib.redirect_stdout(io.StringIO()) as output:
            t.read_status(self.arm)
        self.assertIn('19', output.getvalue())
        self.assertEqual(self.arm.calls, [])

    def test_check_port_does_not_import_sdk(self):
        with mock.patch.object(t, 'probe', return_value=True), \
                mock.patch.dict('sys.modules', {'xarm.wrapper': None}):
            self.assertEqual(t.main(['--check-port']), 0)

    def test_status_entrypoint_never_enables_or_moves(self):
        reports = t.ReportWatch()
        self.arm.connect = mock.Mock(side_effect=lambda: reports.update({}))
        self.arm.disconnect = mock.Mock()
        self.arm.set_timeout = mock.Mock()
        with mock.patch.object(t, 'probe', return_value=True), \
                mock.patch.object(t, 'prepare_arm', return_value=(self.arm, reports)), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(t.main(['--status']), 0)
        self.arm.disconnect.assert_called_once()
        self.assertEqual(self.arm.calls, [])

    def test_pinned_sdk_startup_does_not_connect_or_clear_warning(self):
        with mock.patch('socket.socket.connect', side_effect=AssertionError('Unexpected network')):
            arm, reports = t.prepare_arm('192.168.2.205')
            arm.arm._warn_code = 9
            with mock.patch.object(type(arm.arm), 'clean_warn') as underlying:
                arm.arm.clean_warn()
                underlying.assert_not_called()
            self.assertEqual(arm.warn_code, 9)
            self.assertFalse(arm.connected)
            self.assertFalse(reports.received.is_set())

    def test_stale_reports_stop_stream(self):
        self.arm_on()
        self.frame({'w'})
        self.c.reports = t.ReportWatch()
        self.frame({'w'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_invalid_feedback_and_boolean_status_are_rejected(self):
        for result in (False, True, (False, []), [], [0], [0, 1, 2]):
            with self.assertRaises(RuntimeError):
                t.checked(result, 'Test')
        for value in ([], [1, 2], [float('nan')] * 6, [True] * 6):
            with self.assertRaises(RuntimeError):
                t.vector(value, 'test')


if __name__ == '__main__':
    unittest.main()
