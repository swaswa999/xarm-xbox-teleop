#!/usr/bin/env python3
"""macOS Xbox jogging using UFACTORY's mode-7 online trajectory planner.

Motion approach: xArm-Developer/ufactory_teleop/xbox_teleop.
See THIRD_PARTY_NOTICES.md and LICENSES/ufactory-teleop-BSD-3-Clause.txt.
This adapter uses native macOS input (SDL elsewhere), guarded startup, bounded
targets, and the existing reviewed gripper/connection helpers. It deliberately
does not instantiate upstream UFRobot, which moves and changes settings on init.

--input-only never imports the robot SDK or connects to the robot.
--list-controllers lists detected controllers without robot access.
"""
import argparse
from dataclasses import dataclass
import ipaddress
import json
import math
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

from xarm_teleop import (
    Teleop, Input, Cancelled, checked, vector, prepare_arm, probe, speed_limit,
    acquire_control_lock,
)

MAX_SPEED = 230.0
Z_SPEED_SCALE = 0.75
MAX_ACCELERATION = 1000.0
LOOKAHEAD_MM = 60.0
LOOKAHEAD_DEGREES = 8.0
# Mode 7 takes Cartesian planner speed/acceleration even for pure rotation.
# is_radian changes orientation coordinates only, not these planner parameters.
ROTATION_PLANNER_SPEED = 50.0
PERIOD = 1 / 30
WINDOW_MAX_AGE = 0.10
INPUT_MAX_AGE = 0.15
REPORT_MAX_AGE = 0.5  # Rich reports on this controller arrive every 200 ms.
FAST_REPORT_MAX_AGE = 0.15
RELEASE_STOP_TIMEOUT = 2.0


@dataclass(frozen=True)
class Feedback:
    pose: tuple
    mode: int
    state: int
    updated: float


def parse_feedback(packet, received):
    """UFACTORY real-time report: big-endian length, little-endian floats."""
    if len(packet) not in (87, 135) or int.from_bytes(packet[:4], 'big') != len(packet):
        raise ValueError("Unexpected real-time robot report.")
    state, mode = packet[4] & 15, packet[4] >> 4
    pose = struct.unpack_from('<6f', packet, 35)
    if state not in range(10) or mode not in range(12) or not all(map(math.isfinite, pose)):
        raise ValueError("Invalid real-time robot feedback.")
    return Feedback(tuple(pose[:3]) + tuple(map(math.degrees, pose[3:])),
                    mode, state, received)


class FastFeedback:
    """Read-only port 30003 receiver; never sends a payload or robot command."""
    def __init__(self, ip):
        self.latest = None
        self.error = None
        self.closed = False
        self.received = threading.Event()
        self.socket = socket.create_connection((ip, 30003), timeout=2)
        self.socket.settimeout(0.1)
        self.thread = threading.Thread(target=self.consume, daemon=True)
        self.thread.start()

    def consume(self):
        buffer = b''
        try:
            while not self.closed:
                try:
                    chunk = self.socket.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    raise RuntimeError("Real-time robot feedback disconnected.")
                buffer += chunk
                while len(buffer) >= 4:
                    size = int.from_bytes(buffer[:4], 'big')
                    if size not in (87, 135):
                        raise ValueError("Unsupported real-time robot report.")
                    if len(buffer) < size:
                        break
                    self.latest = parse_feedback(buffer[:size], time.monotonic())
                    self.received.set()
                    buffer = buffer[size:]
        except Exception as exc:
            if not self.closed:
                self.error = str(exc)

    def check(self):
        sample = self.latest
        if self.closed or self.error:
            raise RuntimeError(self.error or "Real-time feedback closed.")
        if sample is None or time.monotonic() - sample.updated > FAST_REPORT_MAX_AGE:
            raise RuntimeError("Real-time robot feedback is stale. Teleop disarmed.")
        return sample

    def close(self):
        self.closed = True
        self.socket.close()
        self.thread.join(timeout=1)


def base_rotation_target(pose, rotation):
    """Compose a base-frame axis-angle increment with the measured RPY pose."""
    roll, pitch, yaw = (math.radians(v) / 2 for v in pose[3:])
    cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
    # Rz(yaw) Ry(pitch) Rx(roll), as used by xArm Cartesian RPY reports.
    w, x, y, z = (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                  cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)
    norm = math.sqrt(sum(v*v for v in rotation))
    angle = math.radians(LOOKAHEAD_DEGREES)
    a = math.cos(angle / 2)
    b, c, d = (v / norm * math.sin(angle / 2) for v in rotation)
    q = (a*w-b*x-c*y-d*z, a*x+b*w+c*z-d*y,
         a*y-b*z+c*w+d*x, a*z+b*y-c*x+d*w)
    length = math.sqrt(sum(v*v for v in q))
    q = tuple(v / length for v in q)
    if q[0] < 0:
        q = tuple(-v for v in q)
    sine = math.sqrt(sum(v*v for v in q[1:]))
    rotvec = ((0.0, 0.0, 0.0) if sine < 1e-12 else
              tuple(v * (2 * math.atan2(sine, q[0])) / sine for v in q[1:]))
    return list(pose[:3]) + list(rotvec)


def shaped(value, deadzone=0.15):
    """Continuous response through the deadzone, with finer control near center."""
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1.001:
        raise RuntimeError("Invalid controller axis.")
    magnitude = min(1.0, max(0.0, (abs(value) - deadzone) / (1 - deadzone)))
    return math.copysign(0.35 * magnitude + 0.65 * magnitude**3, value)


@dataclass(frozen=True)
class Pad:
    connected: bool = False
    name: str = "No controller detected"
    xyz: tuple = (0.0, 0.0, 0.0)
    grip: int = 0
    precision: bool = False
    buttons: frozenset = frozenset()
    focused: bool = False
    updated: float = 0.0
    rxyz: tuple = (0.0, 0.0, 0.0)

    @property
    def motion_centered(self):
        return (not any(self.xyz + self.rxyz) and not self.grip
                and not self.buttons & {'x', 'b', 'lb', 'rb', 'lt', 'rt', 'dleft', 'dright'})

    @property
    def neutral(self):
        return self.motion_centered


@dataclass
class ReleaseStop:
    """Permission to finish only our own acknowledged deceleration stop."""
    started: float
    centered: bool
    resume_at: float | None = None


def robot_configuration(arm):
    """Settings that must not change during a teleop session."""
    load = arm.tcp_load
    return (tuple(arm.tcp_offset), tuple(arm.world_offset),
            (load[0], tuple(load[1])), arm.collision_sensitivity)


def demand(pad):
    axes = pad.xyz + pad.rxyz
    # Arm and gripper I/O are separate; grip reads must not delay arm stopping.
    # Translation and rotation use separate planner speed caps.
    # Conflicting groups stop rather than selecting one.
    if (any(axes) and pad.grip) or (any(pad.xyz) and any(pad.rxyz)):
        return (0.0,) * 6, 0
    length = math.sqrt(sum(v*v for v in axes))
    # Scale Z after diagonal normalization, preserving the X/Y components of
    # the previous requested velocity even when a bumper and stick are combined.
    return tuple(v / max(1.0, length) * (Z_SPEED_SCALE if i == 2 else 1.0)
                 for i, v in enumerate(axes)), pad.grip


class XboxTeleop(Teleop):
    gripper_speed = 1000
    gripper_step = 20.0  # One outstanding increment; never accumulate targets.

    def __init__(self, arm, reports=None, clock=time.monotonic, feedback=None,
                 robot_ip="192.168.2.205"):
        super().__init__(arm, reports, clock)
        self.robot_ip = robot_ip
        self.feedback = feedback
        self.pad = Pad()
        self.drill = None
        self.speed = MAX_SPEED
        self.acceleration = MAX_ACCELERATION
        self.moving = False
        self.orientation = None
        self.motion_group = None
        self.release_stop = None
        self.configuration = None
        self.last_send = -math.inf
        self.last_grip_log = -math.inf
        self.notice = ""
        self.notice_until = 0.0
        self.message = "DISARMED. Release all controls, then press Menu / E."

    def publish_pad(self, pad):
        xyz, grip = demand(pad)
        keys = {'w'} if any(xyz) else ({'left'} if grip > 0 else {'right'} if grip < 0 else set())
        with self.lock:
            self.pad = pad
            # Window heartbeat and native controller sample have separate clocks
            # of receipt. Do not apply the window's 100 ms limit to a pad sample
            # that is still inside the controller's 150 ms freshness limit.
            self.input = Input(frozenset(keys), pad.focused, self.clock())
        if self.drill:
            self.drill.publish(pad)
        self.wake.set()

    def require_input(self):
        with self.lock:
            snapshot, pad = self.input, self.pad
        now = self.clock()
        if not snapshot.focused:
            raise RuntimeError("Control window lost focus. Press Menu / E to enable again.")
        if not pad.connected:
            raise RuntimeError("Controller disconnected. Reconnect, then press Menu / E.")
        window_age, pad_age = now - snapshot.updated, now - pad.updated
        if not 0 <= window_age <= WINDOW_MAX_AGE:
            raise RuntimeError(f"Control window updates stopped ({window_age * 1000:.0f} ms). "
                               "Press Menu / E to enable again.")
        if not 0 <= pad_age <= INPUT_MAX_AGE:
            raise RuntimeError(f"Controller input stopped ({pad_age * 1000:.0f} ms). "
                               "Press Menu / E to enable again.")
        if (len(pad.xyz) != 3 or len(pad.rxyz) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v)
                       or abs(v) > 1 for v in pad.xyz + pad.rxyz)
                or type(pad.grip) is not int or pad.grip not in (-1, 0, 1)):
            raise RuntimeError("Invalid controller input.")
        return snapshot

    def request(self, action):
        if not self.pad.neutral:
            self.message = ("Center sticks; release bumpers, triggers, X and B before gripper setup."
                            if action == 'gripper' else
                            "Center sticks and release all motion buttons before enabling.")
            accepted = False
        else:
            accepted = super().request(action)
        if not accepted:
            self.notice, self.notice_until = self.message, self.clock() + 4
        print(f"Setup {action}: {'accepted' if accepted else self.message}", flush=True)
        return accepted

    def health(self, active=False):
        if not active:
            return super().health()
        if not self.arm.connected:
            raise RuntimeError("Robot disconnected. Teleop disarmed.")
        if self.reports:
            self.reports.check()
            if time.monotonic() - self.reports.updated > REPORT_MAX_AGE:
                raise RuntimeError("Robot feedback is stale. Teleop disarmed.")
        allowed = (0, 1, 2, 6, 4, 5) if self.release_stop is not None else (0, 1, 2, 6)
        if self.feedback:
            sample = self.feedback.check()
            if sample.mode != 7 or sample.state not in allowed:
                raise RuntimeError(f"Robot state/mode changed (fast: mode {sample.mode}, "
                                   f"state {sample.state}). Press Menu / E after resolving it.")
        # These values are updated by the rich-report receiver. Avoid synchronous
        # network reads in the 30 Hz motion loop.
        if self.arm.error_code or self.arm.warn_code:
            raise RuntimeError(f"Robot error {self.arm.error_code}, warning {self.arm.warn_code}.")
        if self.configuration is not None and robot_configuration(self.arm) != self.configuration:
            raise RuntimeError("Robot tool, payload, coordinates, or collision settings changed. "
                               "Teleop disarmed.")
        if self.arm.axis != 6 or self.arm.mode != 7 or self.arm.state not in allowed:
            raise RuntimeError(f"Robot state/mode changed (SDK: mode {self.arm.mode}, "
                               f"state {self.arm.state}). Press Menu / E after resolving it.")
        return self.arm.state

    def enable(self, generation):
        state = self.health()
        if self.feedback and self.feedback.check().state not in (0, 2, 4, 5):
            raise RuntimeError("Robot is moving or paused. Stop Studio motion first.")
        if tuple(self.arm.version_number) < (2, 5, 0):
            raise RuntimeError("This adapter requires firmware 2.5.0 or newer.")
        # State 5 is the stopped MODE_CHANGED state seen after leaving Studio.
        # Accept it only for this explicit enable, never while already armed.
        if state not in (0, 2, 4, 5):
            raise RuntimeError("Robot is moving or paused. Stop Studio motion first.")
        if checked(self.arm.get_cmdnum(), "Read queue") != 0:
            raise RuntimeError("Robot has queued commands. Stop the other controller first.")
        reduced = checked(self.arm.get_reduced_states(is_radian=False), "Read reduced limits")
        self.speed = min(MAX_SPEED, speed_limit(self.arm, reduced), speed_limit(self.arm))
        limits = self.arm.tcp_acc_limit
        if (not isinstance(limits, (tuple, list)) or len(limits) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in limits)
                or not 0 < limits[0] <= limits[1] or limits[0] > MAX_ACCELERATION):
            raise RuntimeError("Invalid controller acceleration limits.")
        self.acceleration = min(MAX_ACCELERATION, limits[1])
        vector(checked(self.arm.get_position(is_radian=False), "Read pose"), "pose")
        configuration = robot_configuration(self.arm)
        if self.feedback:
            self.feedback.check()
        self.valid(generation)
        if not self.pad.neutral:
            raise RuntimeError("Controls moved during enable. Center them and press Menu / E.")
        self.send(generation, self.arm.set_state, 4, claim=True)
        self.send(generation, self.arm.motion_enable, enable=True)
        self.send(generation, self.arm.set_mode, 7)
        self.send(generation, self.arm.set_state, 0)
        if self.feedback or self.reports:
            # Both streams must reflect this enable. The fast report can already
            # say mode 7 while the 200 ms rich report still caches the old mode.
            # No position commands are sent while either stream catches up.
            enabled_at, deadline = time.monotonic(), time.monotonic() + 0.5
            while True:
                self.valid(generation)
                if not self.arm.connected:
                    raise RuntimeError("Robot disconnected during enable.")
                if self.reports:
                    self.reports.check()
                sample = self.feedback.check() if self.feedback else None
                fast_ready = (sample is None or
                              (sample.updated > enabled_at and sample.mode == 7
                               and sample.state in (0, 2)))
                rich_ready = self.reports is None or self.reports.updated > enabled_at
                if self.arm.error_code or self.arm.warn_code:
                    raise RuntimeError(f"Robot error {self.arm.error_code}, warning {self.arm.warn_code}.")
                if robot_configuration(self.arm) != configuration:
                    raise RuntimeError("Robot settings changed during enable. Teleop disarmed.")
                if fast_ready and rich_ready and self.arm.mode == 7 and self.arm.state in (0, 2):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("Robot did not confirm standby on both feedback streams after enable.")
                time.sleep(0.01)
        with self.lock:
            self.valid(generation)
            self.armed = True
            self.neutral_required = True
            self.moving = False
            self.orientation = None
            self.motion_group = None
            self.release_stop = None
            self.configuration = configuration
            self.message = "ENABLED. Use motion controls directly. Release them to stop."

    def enable_gripper(self, generation):
        super().enable_gripper(generation)
        with self.lock:
            self.valid(generation)
            # Any motion input changed during setup still requires a reset.
            self.neutral_required = not self.pad.motion_centered
        self.message = "Gripper enabled. Hold X to open / B to close; release to stop steps."
        self.notice, self.notice_until = self.message, self.clock() + 4
        self.last_grip_log = -math.inf
        print(f"Gripper ready: position {self.gripper_position:.0f}.", flush=True)

    def grip(self, generation, direction):
        if not self.gripper_ready:
            self.message = "Release X / B and press Y / G once to enable the gripper."
            return
        super().grip(generation, direction)
        if self.clock() - self.last_grip_log >= 1:
            print(f"Gripper {'open' if direction > 0 else 'close'}: "
                  f"position {self.gripper_position}, target {self.gripper_target}; "
                  f"{self.message}", flush=True)
            self.last_grip_log = self.clock()

    def stop(self, reason="Stopped. Release controls, then press Menu / E."):
        if self.drill:
            self.drill.disarm()
        print("Teleop:", reason, flush=True)
        self.release_stop = None
        super().stop(reason)

    def zero_velocity(self, generation):
        # Mode 7 uses native deceleration stop, not a velocity-mode command.
        if self.moving:
            self.send(generation, self.arm.set_state, 6)
            self.valid(generation)
            self.release_stop = ReleaseStop(self.clock(), self.pad.motion_centered)
            self.moving = False
            self.orientation = None
            self.motion_group = None
            print(f"Release stop acknowledged: SDK state {self.arm.state}.", flush=True)

    def finish_release_stop(self, generation):
        """Wait for rest, then restore standby only for a new held demand.

        This robot reports state 5 after set_state(6) in mode 7; others report 4.
        Never treat an unrelated stopped state as permission to resume, or turn
        servos back on here. Changed critical settings invalidate the session.
        """
        pending = self.release_stop
        if pending is None:
            return True
        if self.pad.motion_centered:
            pending.centered = True
        sample = self.feedback.check() if self.feedback else None
        state = sample.state if sample else self.arm.state
        updated = sample.updated if sample else self.clock()
        if pending.resume_at is None:
            if updated < pending.started or state in (1, 6) or self.arm.state in (1, 6):
                if self.clock() - pending.started > RELEASE_STOP_TIMEOUT:
                    raise RuntimeError("Deceleration did not finish. Teleop disarmed.")
                self.message = "Finishing deceleration."
                return False
            axes, grip = demand(self.pad)
            if not pending.centered:
                self.message = "Center motion controls once, then move again. Menu is not needed."
                return False
            if not any(axes) and not grip:
                self.message = "Ready. Use motion controls to move again."
                return False
            if self.reports and self.reports.updated <= pending.started:
                self.message = "Waiting for settings feedback after deceleration."
                return False
            # A fresh operator demand may resume our release stop. Read faults
            # synchronously because the rich report can lag by 200 ms.
            err, warn = checked(self.arm.get_err_warn_code(), "Read errors before resuming")
            if err or warn:
                raise RuntimeError(f"Robot error {err}, warning {warn}. Resolve in Studio.")
            if checked(self.arm.get_cmdnum(), "Read queue before resuming") != 0:
                raise RuntimeError("Unexpected queued motion. Teleop disarmed.")
            state = checked(self.arm.get_state(), "Read stopped state")
            self.health(active=True)
            self.valid(generation)
            axes, grip = demand(self.pad)
            if self.release_stop is not pending or (not any(axes) and not grip):
                return False
            if state in (4, 5):
                self.send(generation, self.arm.set_state, 0)
                self.valid(generation)
                pending.resume_at = self.clock()
                print("Release stop: standby requested for new held input.", flush=True)
            elif state in (0, 2):
                self.release_stop = None
                return True
            else:
                raise RuntimeError(f"Unexpected state {state} after deceleration.")
        sample = self.feedback.check() if self.feedback else None
        state = sample.state if sample else self.arm.state
        updated = sample.updated if sample else self.clock()
        if updated >= pending.resume_at and state in (0, 2) and self.arm.state in (0, 2):
            self.release_stop = None
            return True
        if self.clock() - pending.resume_at > 0.5:
            raise RuntimeError("Standby was not confirmed after release stop.")
        self.message = "Waiting for standby confirmation."
        return False

    def _stop_now(self):
        stopping = self.stopping
        super()._stop_now()
        if stopping:
            self.moving = False
            self.orientation = None
            self.motion_group = None
            self.release_stop = None

    def tick(self):
        try:
            self._stop_now()
            if self.closed:
                return
            with self.lock:
                pending, self.pending = self.pending, None
                self.processing = pending is not None
                generation = self.generation
            if pending:
                action, generation = pending
                try:
                    if action == 'enable':
                        self.enable(generation)
                    else:
                        self.enable_gripper(generation)
                finally:
                    with self.lock:
                        self.processing = False
                        self.busy = self.pending is not None
                return
            if not self.armed:
                return
            self.require_input()
            self.health(active=True)
            self.valid(generation)
            if self.drill and not self.drill.ready:
                self.message = "Waiting for ID 8 drill. Release all movement controls."
                return
            pad = self.pad
            if self.neutral_required:
                if not pad.neutral:
                    self.message = "Release all controls once before moving."
                    return
                self.neutral_required = False
            self.speed = min(self.speed, speed_limit(self.arm))
            if not self.finish_release_stop(generation):
                return
            # Stop/resume checks may involve I/O; use the latest held controls.
            pad = self.pad
            axes, grip = demand(pad)
            xyz, rotation = axes[:3], axes[3:]
            if any(axes):
                if self.clock() - self.last_send < PERIOD:
                    return
                group = 'rotation' if any(rotation) else 'translation'
                if self.motion_group and group != self.motion_group:
                    self.zero_velocity(generation)
                    return
                pose = vector(list(self.feedback.check().pose) if self.feedback
                              else self.arm.position, "reported pose")
                magnitude = math.sqrt(sum(v*v for v in axes))
                factor = magnitude * (0.2 if pad.precision else 1.0)
                if group == 'rotation':
                    # A genuine base-frame rotation, not addition to Euler angles.
                    target = base_rotation_target(pose, rotation)
                    rotation_limit = ROTATION_PLANNER_SPEED * min(1.0, self.speed / MAX_SPEED)
                    speed = rotation_limit * factor
                    if speed < self.arm.tcp_speed_limit[0]:
                        self.zero_velocity(generation)
                        return
                    method = self.arm.set_position_aa
                    args = (target,)
                    options = dict(is_radian=True, is_tool_coord=False, relative=False,
                                   mvacc=self.acceleration)
                    display_speed = f"rotation {factor:.0%}"
                else:
                    if self.orientation is None:
                        self.orientation = tuple(pose[3:])
                    speed = self.speed * factor
                    if speed < self.arm.tcp_speed_limit[0]:
                        self.zero_velocity(generation)
                        return
                    target = [pose[i] + xyz[i] / magnitude * LOOKAHEAD_MM for i in range(3)]
                    target += list(self.orientation)
                    method, args = self.arm.set_position, tuple(target)
                    options = dict(mvacc=self.acceleration, is_radian=False)
                    display_speed = f"{speed:.0f} mm/s"
                self.valid(generation)
                self.health(active=True)
                latest_axes, latest_grip = demand(self.pad)
                if (latest_axes, latest_grip) != (axes, grip):
                    # Analog noise/updates are not a release. Skip an outdated
                    # sample; only an actual release or group change needs stop.
                    if not any(latest_axes) or bool(any(latest_axes[:3])) != bool(any(xyz)):
                        self.zero_velocity(generation)
                    return
                self.send(generation, method, *args, speed=speed, wait=False, **options)
                if not self.moving:
                    print(f"Motion {group}: axes {tuple(round(v, 3) for v in axes)}, "
                          f"buttons {sorted(pad.buttons)}, "
                          f"planner speed {speed:.3f}, acceleration {options['mvacc']:.1f}, "
                          f"start pose {tuple(round(v, 3) for v in pose)}, "
                          f"target {tuple(round(v, 4) for v in target)}.", flush=True)
                self.moving = True
                self.motion_group = group
                self.last_send = self.clock()
                self.message = f"Moving · {display_speed} requested. Release controls to stop."
            else:
                self.zero_velocity(generation)
                if grip:
                    if (self.feedback.check().state if self.feedback else self.arm.state) in (1, 6):
                        self.message = "Waiting for arm motion to stop before gripping."
                    else:
                        self.grip(generation, grip)
                else:
                    self.gripper_blocked_direction = 0
                    self.last_grip_log = -math.inf
                    conflict = ((pad.grip and any(pad.xyz + pad.rxyz))
                                or (any(pad.xyz) and any(pad.rxyz)))
                    self.message = ("Translate, rotate, and grip separately." if conflict
                                    else ("Ready. Use motion controls directly; X opens / B closes." if self.gripper_ready
                                          else "Ready. Use motion controls directly. Y / G enables the gripper."))
        except Cancelled:
            pass
        except Exception as exc:
            self.stop(str(exc))
        finally:
            self._stop_now()

    def close(self):
        try:
            super().close()
        finally:
            if self.drill:
                self.drill.close()

    def start(self):
        if self.drill:
            self.drill.start()
        def run():
            while True:
                self.wake.clear()
                self.tick()
                if self.closed:
                    return
                self.wake.wait(PERIOD)
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()


class PadReader:
    """SDL semantic axes/buttons; no device-specific numeric joystick guesses."""
    def __init__(self, pg, gc):
        self.pg, self.gc = pg, gc
        self.device = None
        self.next_scan = 0.0
        self.note = "Connect your Xbox controller to this Mac."

    def read(self, focused):
        now = time.monotonic()
        if self.device is not None and not self.device.attached():
            self.device.quit()
            self.device = None
            self.next_scan = 0.0
        if self.device is None and now >= self.next_scan:
            self.next_scan = now + 0.5
            matches = [i for i in range(self.gc.get_count()) if self.gc.is_controller(i)]
            if len(matches) == 1:
                self.device = self.gc.Controller(matches[0])
                self.note = "Controller detected."
            elif len(matches) > 1:
                self.note = "Multiple gamepads detected. Connect only the one you want to use."
            else:
                self.note = "No gamepad detected. Pair it in macOS Bluetooth settings."
        if self.device is None:
            return Pad(name=self.note, focused=focused, updated=now)
        p, d = self.pg, self.device
        axes = [d.get_axis(a) for a in (
            p.CONTROLLER_AXIS_LEFTX, p.CONTROLLER_AXIS_LEFTY,
            p.CONTROLLER_AXIS_RIGHTX, p.CONTROLLER_AXIS_RIGHTY, p.CONTROLLER_AXIS_TRIGGERLEFT,
            p.CONTROLLER_AXIS_TRIGGERRIGHT,
        )]
        norm = [max(-1.0, min(1.0, a / 32767)) for a in axes]
        mappings = {
            'menu': p.CONTROLLER_BUTTON_START, 'view': p.CONTROLLER_BUTTON_BACK,
            'b': p.CONTROLLER_BUTTON_B, 'x': p.CONTROLLER_BUTTON_X,
            'a': p.CONTROLLER_BUTTON_A, 'y': p.CONTROLLER_BUTTON_Y,
            'lb': p.CONTROLLER_BUTTON_LEFTSHOULDER, 'rb': p.CONTROLLER_BUTTON_RIGHTSHOULDER,
            'l3': p.CONTROLLER_BUTTON_LEFTSTICK, 'r3': p.CONTROLLER_BUTTON_RIGHTSTICK,
            'dleft': p.CONTROLLER_BUTTON_DPAD_LEFT, 'dright': p.CONTROLLER_BUTTON_DPAD_RIGHT,
        }
        buttons = [name for name, code in mappings.items() if d.get_button(code)]
        # Standardize SDL's downward-positive Y to Apple's upward-positive Y.
        from dataclasses import replace
        packet = dict(version=2, connected=True, name=d.name, buttons=buttons,
                      axes=[norm[0], -norm[1], norm[2], -norm[3],
                            max(0, norm[4]), max(0, norm[5])])
        return replace(native_pad(packet, now), focused=focused)

    def close(self):
        if self.device:
            self.device.quit()


def native_pad(packet, received):
    """Validate Apple GameController messages before accepting any input."""
    if (not isinstance(packet, dict) or packet.get('version') != 2
            or type(packet.get('connected')) is not bool):
        raise ValueError("Invalid native controller message")
    name = str(packet.get('name', 'Xbox controller'))[:120]
    if not packet['connected']:
        return Pad(name=name, updated=received)
    axes = packet.get('axes')
    if (not isinstance(axes, list) or len(axes) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v)
                   or not -1 <= v <= 1 for v in axes)
            or any(v < 0 for v in axes[4:])):
        raise ValueError("Invalid native controller axes")
    buttons = packet.get('buttons')
    if (not isinstance(buttons, list)
            or any(type(v) is not str or v not in
                   ('menu', 'view', 'a', 'b', 'x', 'y', 'lb', 'rb', 'l3', 'r3', 'dleft', 'dright') for v in buttons)):
        raise ValueError("Invalid native controller buttons")
    buttons = frozenset(buttons) | ({'lt'} if axes[4] > 0.15 else set()) \
        | ({'rt'} if axes[5] > 0.15 else set())
    # Apple Y is positive upward; SDL Y is positive downward.
    xyz = (shaped(axes[1]), -shaped(axes[0]), float(('rb' in buttons) - ('lb' in buttons)))
    rotation = (shaped(axes[2]), shaped(axes[3]), shaped(axes[5]) - shaped(axes[4]))
    grip = int('x' in buttons) - int('b' in buttons)
    return Pad(connected=True, name=name, xyz=xyz, grip=grip, precision='r3' in buttons,
               buttons=buttons, updated=received, rxyz=rotation)


class NativePadReader:
    def __init__(self):
        bridge = Path(__file__).parent / 'xbox-support' / 'controller_bridge'
        if not bridge.is_file():
            raise RuntimeError("Native Xbox input helper is missing. Run the setup build.")
        self.latest = Pad(name="Checking Apple controller connection…")
        self.closed = False
        self.seen = set()
        self.process = subprocess.Popen([str(bridge)], stdin=subprocess.DEVNULL,
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        text=True, bufsize=1)
        self.thread = threading.Thread(target=self.consume, daemon=True)
        self.thread.start()

    def consume(self):
        try:
            for line in self.process.stdout:
                if self.closed:
                    break
                try:
                    pad = native_pad(json.loads(line), time.monotonic())
                    self.latest = pad
                    detected = set(pad.buttons)
                    if any(pad.xyz[:2]): detected.add('left stick')
                    if any(pad.rxyz[:2]): detected.add('right stick')
                    if pad.connected and 'connected' not in self.seen:
                        print("Controller connected:", pad.name, flush=True)
                        self.seen.add('connected')
                    for item in sorted(detected - self.seen):
                        print("Input detected:", item, flush=True)
                    self.seen |= detected
                except (ValueError, TypeError):
                    self.latest = Pad(name="Controller input could not be read.", updated=time.monotonic())
        finally:
            self.latest = Pad(name="Controller input helper stopped.", updated=time.monotonic())

    def read(self, focused):
        from dataclasses import replace
        pad = self.latest
        if self.process.poll() is not None or time.monotonic() - pad.updated > INPUT_MAX_AGE:
            return Pad(name="Controller input unavailable. Reconnect or restart the window.",
                       focused=focused, updated=time.monotonic())
        return replace(pad, focused=focused)

    def close(self):
        self.closed = True
        self.process.terminate()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2)
        self.thread.join(timeout=2)
        self.process.stdout.close()


class PadControls:
    def __init__(self):
        self.previous = frozenset()
        self.was_connected = False

    def frame(self, controller, pad, keyboard=frozenset(), lost=False, quit_requested=False):
        buttons = pad.buttons | frozenset(keyboard)
        fresh = buttons - self.previous
        self.previous = buttons
        controller.publish_pad(pad)
        if quit_requested or buttons & {'escape', 'q'}:
            controller.stop("Controller window closed.")
            return False
        if lost or not pad.focused or not pad.connected:
            if controller.owned or controller.armed or controller.busy:
                controller.stop("Controller disconnected or window lost focus. Enable again when ready.")
            self.was_connected = pad.connected
            return True
        if not self.was_connected:
            # Ignore buttons held while a controller is newly attached.
            self.was_connected = True
            return True
        if buttons & {'view', 'space'}:
            if fresh & {'view', 'space'} or controller.armed or controller.busy:
                controller.stop("Stopped. Center controls, then press Menu / E.")
        elif fresh & {'menu', 'e'}:
            controller.request('enable')
        elif fresh & {'y', 'g'}:
            controller.request('gripper')
        return True


def draw(pg, surface, c, pad, fonts, input_only):
    surface.fill('#10181e')
    title, normal, small = fonts
    def label(text, x, y, font=normal, color='#edf4f5'):
        surface.blit(font.render(text, True, color), (x, y))
    has_drill = c is not None and c.drill is not None
    label("xArm 6 + ID 8 drill" if has_drill else "xArm 6 · Xbox control", 28, 20, title)
    label("INPUT CHECK · Robot disconnected" if input_only
          else f"{c.robot_ip} · Base coordinates · Up to {c.speed:.0f} mm/s",
          28, 65, small, '#a1b4c0')
    label(pad.name[:88], 28, 96, small, '#68d6b4' if pad.connected else '#efbd69')
    label("ENABLED" if c and c.armed else "DISARMED", 28, 133, normal, '#efbd69')
    label("MENU / E  Enable arm + drill" if has_drill else
          "MENU / E  Enable arm     Y / G  Enable gripper", 245, 137, small)
    cards = [
        ("LEFT STICK", "Move X / Y", "Forward = +X · Left = +Y"),
        ("RIGHT STICK", "Rotate RX / RY", "Right = +RX · Up = +RY"),
        ("LB / RB", "Z down / up", "Lower / raise · 25% slower"),
        ("LT / RT", "Rotate RZ− / RZ+", "Triggers: negative / positive"),
        ("X / B", "Open / close gripper", "Hold button; release to stop steps"),
        ("VIEW / SPACE", "Stop & disarm", "Center controls; Menu enables again"),
    ]
    for i, (key, action, note) in enumerate(cards):
        x, y = 28 + (i % 2) * 390, 184 + (i // 2) * 115
        pg.draw.rect(surface, '#1c2932', (x, y, 374, 102), border_radius=12)
        label(key, x+16, y+12, normal, '#68d6b4')
        label(action, x+16, y+43, normal)
        label(note, x+16, y+75, small, '#a1b4c0')
    label("D-pad RIGHT / LEFT: drill forward / reverse (hold)" if has_drill else
          "Hold R3: precision       VIEW / SPACE: stop & disarm",
          28, 545, small, '#efbd69')
    for i, (axis, value) in enumerate(zip(('X', 'Y', 'Z', 'RX', 'RY', 'RZ'), pad.xyz + pad.rxyz)):
        x, y = 28 + (i // 3) * 390, 578 + (i % 3) * 24
        label(axis, x, y, small)
        pg.draw.rect(surface, '#1c2932', (x+40, y+3, 300, 13), border_radius=4)
        pg.draw.line(surface, '#a1b4c0', (x+190, y), (x+190, y+20))
        if value:
            start = x+190 if value > 0 else x+190 + int(150 * value)
            pg.draw.rect(surface, '#68d6b4', (start, y+3, max(1, int(abs(value)*150)), 13))
    label("ID 8 drill · speed 2000" if has_drill else
          "Direct controls · no A hold needed", 28, 658, small)
    grip_state = "INPUT ONLY" if input_only else ("READY" if c.gripper_ready else "OFF · press Y / G")
    grip_input = ('open' if pad.grip > 0 else 'close') if pad.grip else 'neutral'
    label(f"Gripper: {grip_state} · {grip_input}", 418, 658, small,
          '#68d6b4' if c and c.gripper_ready else '#efbd69')
    label(("D-pad input: " + ('forward' if 'dright' in pad.buttons and 'dleft' not in pad.buttons
                             else 'reverse' if 'dleft' in pad.buttons and 'dright' not in pad.buttons
                             else 'stopped')) if input_only else c.drill.message if has_drill else
          "Translate, rotate, and grip separately. ESC / Q quits.", 28, 689, small, '#a1b4c0')
    label("Release controls to stop. Keep the physical E-stop within reach.", 28, 713, small, '#a1b4c0')
    message = ("Input check only. Sticks and buttons cannot move the robot." if input_only
               else c.message)
    if c and c.armed and not c.moving and not pad.grip and c.clock() < c.notice_until:
        message = c.notice
    line, rows = "", []
    for word in message.split():
        trial = (line + ' ' + word).strip()
        if small.size(trial)[0] > 760:
            rows.append(line)
            line = word
        else:
            line = trial
    rows.append(line)
    for i, line in enumerate(rows[:3]):
        label(line, 28, 752+i*22, small, '#efbd69')


def window(controller=None, input_only=False, max_frames=None):
    import pygame as pg
    from pygame._sdl2 import controller as gc
    pg.display.init()
    pg.font.init()
    gc.init()
    reader = None
    try:
        surface = pg.display.set_mode((820, 840))
        pg.display.set_caption("xArm Xbox — Input check" if input_only else "xArm Xbox controls")
        fonts = [pg.font.SysFont('Arial', size) for size in (30, 21, 17)]
        reader = NativePadReader() if sys.platform == 'darwin' else PadReader(pg, gc)
        controls = PadControls()
        timer = pg.time.Clock()
        keymap = {pg.K_e: 'e', pg.K_g: 'g', pg.K_SPACE: 'space',
                  pg.K_ESCAPE: 'escape', pg.K_q: 'q'}
        if controller:
            controller.start()
        frames = 0
        while max_frames is None or frames < max_frames:
            events = pg.event.get()
            pad = reader.read(bool(pg.key.get_focused()))
            pressed = pg.key.get_pressed()
            keyboard = {name for key, name in keymap.items() if pressed[key]}
            keyboard |= {keymap[e.key] for e in events if e.type == pg.KEYDOWN and e.key in keymap}
            lost = any(e.type in (pg.WINDOWFOCUSLOST, pg.WINDOWMINIMIZED) for e in events)
            quitting = any(e.type == pg.QUIT for e in events)
            if controller:
                if not controls.frame(controller, pad, keyboard, lost, quitting):
                    break
            elif quitting or keyboard & {'escape', 'q'}:
                break
            draw(pg, surface, controller, pad, fonts, input_only)
            pg.display.flip()
            timer.tick(60)
            frames += 1
    finally:
        if controller:
            controller.close()
        if reader:
            reader.close()
        gc.quit()
        pg.quit()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default='192.168.2.205',
                        type=lambda value: str(ipaddress.IPv4Address(value)))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--input-only', action='store_true')
    mode.add_argument('--list-controllers', action='store_true')
    parser.add_argument('--drill-port', help='USB serial path for the standalone ID 8 drill')
    mode.add_argument('--check-drill', action='store_true', help='Read drill identity only; no motor writes')
    args = parser.parse_args(argv)
    if args.check_drill:
        if not args.drill_port:
            parser.error('--check-drill requires --drill-port')
        from so101_drill import SerialDrill
        drill = SerialDrill(args.drill_port)
        try:
            drill.open()
            print(f'Drill ID 8 verified on {args.drill_port}; no writes sent.')
            return 0
        except Exception as exc:
            print(f'Drill check: {exc}')
            return 1
        finally:
            drill.close()
    if args.input_only:
        window(input_only=True)
        return 0
    if args.list_controllers:
        if sys.platform == 'darwin':
            reader = NativePadReader()
            try:
                time.sleep(1)
                print(reader.read(False))
            finally:
                reader.close()
            return 0
        import pygame as pg
        from pygame._sdl2 import controller as gc
        pg.display.init()
        gc.init()
        try:
            pg.event.pump()
            print("Controllers:", gc.get_count())
            for i in range(gc.get_count()):
                if gc.is_controller(i):
                    pad = gc.Controller(i)
                    print(i, pad.name, pad.get_mapping())
        finally:
            gc.quit()
            pg.quit()
        return 0
    arm = controller = lock = feedback = None
    previous = {}
    try:
        lock = acquire_control_lock()
        if not probe(args.ip):
            return 2
        arm, reports = prepare_arm(args.ip)
        arm.connect()
        arm.set_timeout(0.5)
        if not reports.received.wait(3):
            raise RuntimeError("No robot status reports.")
        reports.check()
        feedback = FastFeedback(args.ip)
        if not feedback.received.wait(2):
            raise RuntimeError("No real-time robot feedback.")
        feedback.check()
        controller = XboxTeleop(arm, reports, feedback=feedback, robot_ip=args.ip)
        if args.drill_port:
            from so101_drill import DrillController, SerialDrill
            controller.drill = DrillController(
                lambda: SerialDrill(args.drill_port),
                lambda: controller.armed and not controller.stopping and not controller.closed,
                controller.stop,
            )
        controller.health()
        print("Xbox controls started DISARMED. No motion or configuration writes at startup.", flush=True)
        def interrupted(*_):
            raise KeyboardInterrupt
        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGTSTP):
            previous[sig] = signal.getsignal(sig)
            signal.signal(sig, interrupted)
        window(controller)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Xbox controls: {exc}", flush=True)
        return 1
    finally:
        if controller and not controller.closed:
            controller.close()
        if arm:
            arm.disconnect()
        if feedback:
            feedback.close()
        if lock:
            lock.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
