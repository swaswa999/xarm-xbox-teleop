"""Offline serial and combined-control tests; no real motor writes."""
from dataclasses import replace
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

import so101_drill as d
import xarm_xbox as x
from test_xarm_teleop import Clock
from test_xarm_xbox import FakeArm


class FakeDrill:
    def __init__(self):
        self.calls = []

    def open(self):
        self.calls.append('open')

    def prepare(self, valid):
        valid()
        self.calls.append('prepare')

    def velocity(self, speed):
        self.calls.append(speed)

    def health(self):
        pass

    def close(self):
        self.calls.append('close')


class DrillTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.motor = FakeDrill()
        self.enabled = False
        self.faults = []
        self.c = d.DrillController(lambda: self.motor, lambda: self.enabled,
                                   self.fault, self.clock)

    def fault(self, message):
        self.faults.append(message)
        self.enabled = False

    def frame(self, buttons=(), **kw):
        pad = x.Pad(connected=True, focused=True, updated=self.clock(),
                    buttons=frozenset(buttons))
        self.c.publish(replace(pad, **kw))
        self.c.tick()

    def enable(self):
        self.enabled = True
        self.frame()
        self.frame()

    def test_startup_and_disarmed_dpad_never_open_serial(self):
        self.frame({'dright'})
        self.c.close()
        self.assertEqual(self.motor.calls, [])

    def test_enable_is_zero_motion_and_requires_released_controls(self):
        self.enabled = True
        self.frame({'dright'})
        self.assertEqual(self.motor.calls, [])
        self.frame()
        self.assertEqual(self.motor.calls, ['open', 'prepare'])
        self.frame({'dright'})
        self.assertEqual(self.motor.calls, ['open', 'prepare'])
        self.frame()
        self.frame({'dright'})
        self.assertEqual(self.motor.calls[-1], 2000)

    def test_directions_release_and_opposing_buttons_stop(self):
        self.enable()
        self.frame({'dright'})
        self.assertEqual(self.motor.calls[-1], 2000)
        self.frame()
        self.assertEqual(self.motor.calls[-1], 0)
        self.frame({'dleft'})
        self.assertEqual(self.motor.calls[-1], -2000)
        self.frame({'dleft', 'dright'})
        self.assertEqual(self.motor.calls[-1], 0)

    def test_reverse_pauses_at_zero_and_can_be_cancelled(self):
        self.enable()
        self.frame({'dright'})
        self.frame({'dleft'})
        self.assertEqual(self.motor.calls[-1], 0)
        self.clock.advance(.10)
        self.frame({'dleft'})
        self.assertNotIn(-2000, self.motor.calls)
        self.clock.advance(.06)
        self.frame({'dleft'})
        self.assertEqual(self.motor.calls[-1], -2000)
        self.frame({'dright'})
        self.c.disarm()
        self.enabled = False
        self.clock.advance(.2)
        self.frame({'dright'})
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_stale_window_and_stale_pad_both_disarm(self):
        for stale_pad in (False, True):
            with self.subTest(stale_pad=stale_pad):
                self.setUp()
                self.enable()
                self.frame({'dright'})
                if stale_pad:
                    self.frame({'dright'}, updated=self.clock()-.16)
                else:
                    self.clock.advance(.11)
                    self.c.tick()
                self.assertEqual(self.motor.calls[-1], 'close')
                self.assertTrue(self.faults)
                count = len(self.motor.calls)
                self.frame({'dright'})
                self.assertEqual(len(self.motor.calls), count)

    def test_focus_loss_disconnect_and_view_stop(self):
        for fields in ({'focused': False}, {'connected': False},
                       {'buttons': frozenset({'view', 'dright'})}):
            with self.subTest(fields=fields):
                self.setUp()
                self.enable()
                self.frame({'dright'})
                pad = x.Pad(connected=True, focused=True, updated=self.clock(),
                            buttons=frozenset({'dright'}))
                self.c.publish(replace(pad, **fields))
                self.c.tick()
                self.assertEqual(self.motor.calls[-1], 'close')
                self.assertFalse(self.enabled)

    def test_arm_disarm_closes_drill_without_new_ui_sample(self):
        self.enable()
        self.frame({'dright'})
        self.enabled = False
        self.c.tick()
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_stop_during_connect_never_prepares_motor(self):
        self.enabled = True
        self.motor.open = lambda: self.c.disarm()
        self.frame()
        self.assertNotIn('prepare', self.motor.calls)
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_serial_failure_disarms_and_does_not_retry(self):
        self.enable()
        self.motor.velocity = Mock(side_effect=OSError('USB lost'))
        self.frame({'dright'})
        self.assertFalse(self.enabled)
        self.assertIn('USB lost', self.faults[0])
        self.frame({'dright'})
        self.motor.velocity.assert_called_once()

    def test_close_failure_reports_unconfirmed_stop(self):
        self.enable()
        self.motor.close = Mock(side_effect=OSError('stop unconfirmed'))
        self.enabled = False
        self.c.tick()
        self.assertIn('stop unconfirmed', self.faults[0])

    def test_running_drill_polls_feedback_without_direction_changes(self):
        self.enable()
        self.frame({'dright'})
        self.motor.health = Mock(side_effect=OSError('USB disconnected'))
        self.clock.advance(.10)
        self.frame({'dright'})
        self.assertFalse(self.enabled)
        self.assertIn('USB disconnected', self.faults[0])
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_worker_exits_and_stops_on_close(self):
        self.enable()
        self.frame({'dright'})
        self.c.start()
        self.c.close()
        self.assertFalse(self.c.worker.is_alive())
        self.assertEqual(self.motor.calls[-1], 'close')


class SerialTests(unittest.TestCase):
    def setUp(self):
        self.h = d.SerialDrill('/fake')
        self.h.port = Mock(is_open=True)
        self.h.packet = Mock()
        self.h.packet.write1ByteTxRx.return_value = (0, 0)
        self.h.packet.write2ByteTxRx.return_value = (0, 0)
        self.h.packet.read1ByteTxRx.return_value = (1, 0, 0)

    def test_zero_velocity_precedes_torque_enable_and_no_id_write(self):
        events = []
        self.h.write = lambda a, v, size=1: events.append((a, v, size))
        self.h.prepare(lambda: None)
        self.assertLess(events.index((46, 0, 2)), events.index((40, 1, 1)))
        self.assertFalse(any(a in (5, 9, 11, 31) for a, _, _ in events))

    def test_cancelled_preparation_does_not_enable_torque(self):
        valid = Mock(side_effect=[None, RuntimeError('cancelled')])
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            self.h.prepare(valid)
        self.assertNotIn(((self.h.port, 8, 40, 1),),
                         self.h.packet.write1ByteTxRx.call_args_list)

    def test_reverse_encoding_and_speed_bounds(self):
        self.h.velocity(-2000)
        self.h.packet.write2ByteTxRx.assert_called_with(self.h.port, 8, 46, 34768)
        with self.assertRaises(ValueError):
            self.h.velocity(2001)

    def test_read_only_close_never_writes_motor(self):
        self.h.close()
        self.h.packet.write1ByteTxRx.assert_not_called()
        self.h.packet.write2ByteTxRx.assert_not_called()

    def test_arm_bus_blocks_drill_setup_without_writes(self):
        self.h.packet.ping.side_effect = lambda port, mid: (777, 0, 0)
        self.h.port.getBaudRate.return_value = 1000000
        sdk = SimpleNamespace(PortHandler=lambda path: self.h.port,
                              PacketHandler=lambda protocol: self.h.packet)
        with patch.dict('sys.modules', scservo_sdk=sdk), patch('fcntl.ioctl'):
            with self.assertRaisesRegex(RuntimeError, 'only the ID 8 drill'):
                self.h.open()
        self.h.close()
        self.h.packet.write1ByteTxRx.assert_not_called()
        self.h.packet.write2ByteTxRx.assert_not_called()

    def test_missing_id8_blocks_setup_without_writes(self):
        self.h.packet.ping.return_value = (0, -6, 0)
        self.h.port.getBaudRate.return_value = 1000000
        sdk = SimpleNamespace(PortHandler=lambda path: self.h.port,
                              PacketHandler=lambda protocol: self.h.packet)
        with patch.dict('sys.modules', scservo_sdk=sdk), patch('fcntl.ioctl'):
            with self.assertRaisesRegex(RuntimeError, 'No healthy STS3215'):
                self.h.open()
        self.h.close()
        self.h.packet.write1ByteTxRx.assert_not_called()

    def test_read_only_open_claims_port_and_checks_solo_motor(self):
        self.h.packet.ping.side_effect = lambda port, mid: ((777, 0, 0) if mid == 8 else (0, -6, 0))
        self.h.port.getBaudRate.return_value = 1000000
        sdk = SimpleNamespace(PortHandler=lambda path: self.h.port,
                              PacketHandler=lambda protocol: self.h.packet)
        with patch.dict('sys.modules', scservo_sdk=sdk), patch('fcntl.ioctl') as exclusive:
            self.h.open()
        exclusive.assert_called_once()
        self.h.close()
        self.h.packet.write1ByteTxRx.assert_not_called()
        self.h.packet.write2ByteTxRx.assert_not_called()

    def test_failed_zero_still_attempts_torque_off_and_closes(self):
        self.h.claimed = True
        self.h.packet.write2ByteTxRx.return_value = (-6, 0)
        self.h.packet.read1ByteTxRx.return_value = (0, 0, 0)
        with self.assertRaisesRegex(RuntimeError, 'unconfirmed'):
            self.h.close()
        self.h.packet.write1ByteTxRx.assert_called_with(self.h.port, 8, 40, 0)
        self.h.port.closePort.assert_called_once()


class CombinedTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.arm = FakeArm()
        self.c = x.XboxTeleop(self.arm, clock=self.clock)
        self.motor = FakeDrill()
        self.c.drill = d.DrillController(lambda: self.motor, lambda: self.c.armed,
                                        self.c.stop, self.clock)
        self.controls = x.PadControls()
        self.frame()

    def frame(self, buttons=(), xyz=(0, 0, 0), keyboard=()):
        pad = x.Pad(connected=True, focused=True, updated=self.clock(), xyz=xyz,
                    buttons=frozenset(buttons))
        self.controls.frame(self.c, pad, keyboard)
        self.c.tick()
        self.c.drill.tick()

    def enable(self):
        self.frame({'menu'})
        self.frame()

    def test_dpad_mapping_in_native_messages_does_not_move_arm(self):
        pad = x.native_pad(dict(version=2, connected=True, axes=[0]*6,
                                buttons=['dright']), self.clock())
        self.assertEqual(d.direction(pad), 1)
        self.assertEqual(x.demand(pad), ((0.,)*6, 0))
        self.assertFalse(pad.neutral)

    def test_arm_and_drill_can_operate_together_and_space_stops_both(self):
        self.enable()
        self.frame({'dright'}, xyz=(1, 0, 0))
        self.assertEqual(self.motor.calls[-1], 2000)
        self.assertTrue(any(c[0] == 'position' for c in self.arm.calls))
        self.frame({'dright'}, keyboard={'space'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.motor.calls[-1], 'close')
        self.assertEqual(self.arm.calls[-1], ('state', 4))

    def test_held_drill_blocks_menu_enable(self):
        self.frame({'menu', 'dright'})
        self.assertFalse(self.c.armed)
        self.assertEqual(self.motor.calls, [])

    def test_arm_does_not_move_until_drill_is_ready(self):
        pad = x.Pad(connected=True, focused=True, updated=self.clock())
        self.controls.frame(self.c, pad)
        self.controls.frame(self.c, replace(pad, buttons=frozenset({'menu'})))
        self.c.tick()  # Arm enable, drill worker has not prepared yet.
        self.controls.frame(self.c, replace(pad, xyz=(1, 0, 0)))
        self.c.tick()
        self.assertFalse(any(call[0] == 'position' for call in self.arm.calls))
        self.assertIn('Waiting for ID 8', self.c.message)

    def test_drill_fault_disarms_arm_and_arm_fault_stops_drill(self):
        self.enable()
        self.motor.velocity = Mock(side_effect=OSError('USB lost'))
        self.frame({'dright'})
        self.c.tick()
        self.assertFalse(self.c.armed)
        self.assertEqual(self.arm.calls[-1], ('state', 4))
        self.setUp()
        self.enable()
        self.frame({'dright'})
        self.arm.error = 24
        self.c.tick()
        self.c.drill.tick()
        self.assertFalse(self.c.armed)
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_input_only_with_drill_flag_never_opens_serial_or_robot(self):
        with patch.object(x, 'window') as window, patch.object(d.SerialDrill, 'open') as serial, \
                patch.object(x, 'prepare_arm') as arm:
            self.assertEqual(x.main(['--input-only', '--drill-port', '/fake']), 0)
        window.assert_called_once_with(input_only=True)
        serial.assert_not_called()
        arm.assert_not_called()


if __name__ == '__main__':
    unittest.main()
