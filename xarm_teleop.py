#!/usr/bin/env python3
"""Hold-to-move keyboard control for a UFACTORY xArm 6.

--check-port opens TCP 502 and closes it without sending a payload.
--status reads controller status without enabling, stopping, or moving it.
The control window starts disarmed. E enables; G enables the gripper.
Hold WASD for X/Y, Up/Down for Z, Left/Right for gripper open/close.
Release stops arm velocity; a gripper increment of at most 5 pulses may finish.
No homing, error clearing, brake release, or safety-setting changes.
Keyboard stop is not safety-rated. Keep the physical E-stop within reach.
SDK: https://github.com/xArm-Developer/xArm-Python-SDK
"""
import argparse
from dataclasses import dataclass
import errno
import ipaddress
import math
from pathlib import Path
import signal
import socket
import sys
import threading
import time


def acquire_control_lock():
    """Only one of the local keyboard/Xbox control windows may own the arm."""
    import fcntl
    handle = open(Path(__file__).with_name('.xarm-control.lock'), 'a')
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as exc:
        handle.close()
        raise RuntimeError("Another teleop window is open. Close it before starting this one.") from exc
    return handle


def checked(result, operation):
    if isinstance(result, (tuple, list)):
        if len(result) != 2:
            raise RuntimeError(f"{operation}: malformed SDK response")
        code, value = result
    else:
        code, value = result, None
    if type(code) is not int or code != 0:
        raise RuntimeError(f"{operation}: SDK code {code}")
    return value


def vector(value, label, allow_seventh=False):
    lengths = (6, 7) if allow_seventh else (6,)
    if (not isinstance(value, (tuple, list)) or len(value) not in lengths
            or not all(type(v) in (int, float) and math.isfinite(v) for v in value)):
        raise RuntimeError(f"Invalid {label} feedback")
    return list(value[:6])


def probe(ip):
    print(f"TCP {ip}:502: connection only, no payload.", flush=True)
    try:
        with socket.create_connection((ip, 502), timeout=3):
            pass
    except OSError as exc:
        print(f"TCP check failed: {exc}")
        if exc.errno in (errno.EPERM, errno.EACCES):
            print("Local permissions blocked the connection. Robot reachability is unknown.")
        else:
            print("Check controller power, Ethernet, IP address, and the Mac's network route.")
        return False
    print("TCP connection succeeded. No robot command was sent.")
    return True


class ReportWatch:
    """Track receipt of controller reports, including mode and tool setup."""
    def __init__(self):
        self.received = threading.Event()
        self.updated = 0.0

    def update(self, _report):
        self.updated = time.monotonic()
        self.received.set()

    def check(self):
        if not self.received.is_set() or time.monotonic() - self.updated > 0.5:
            raise RuntimeError("Controller reports are stale; teleop disarmed")


def prepare_arm(ip):
    from xarm.wrapper import XArmAPI
    from xarm.version import __version__

    # These startup internals have been reviewed for this version specifically.
    if __version__ != '1.18.4':
        raise RuntimeError("Use the reviewed SDK: xarm-python-sdk==1.18.4")
    arm = XArmAPI(
        ip, is_radian=False, do_not_open=True, report_type='rich',
        check_robot_sn=False, timed_comm=False, baud_checkset=False,
        # Prevent SDK-side indefinite waits from holding the send lock ahead
        # of a stop. Teleop checks state/queue; controller limits remain active.
        check_is_pause=False, check_cmdnum_limit=False,
    )
    # SDK 1.18.4 may call clean_warn() inside connect(). Suppress that implicit
    # write on this instance; health() still refuses all errors and warnings.
    # Do not alter the installed SDK or the controller's warning state.
    arm.arm.clean_warn = lambda *args, **kwargs: 0
    reports = ReportWatch()
    arm.register_report_callback(reports.update)
    return arm, reports


def read_status(arm):
    # Strictly getters; this path never acquires control or sends set_state.
    print(f"Firmware: {arm.version} | axes: {arm.axis} | mode: {arm.mode}")
    print("State:", checked(arm.get_state(), "Read state"))
    print("Errors/warnings:", checked(arm.get_err_warn_code(), "Read errors"))
    print("Queued commands:", checked(arm.get_cmdnum(), "Read queue"))
    print("Pose:", checked(arm.get_position(is_radian=False), "Read pose"))
    print("Joints:", checked(arm.get_servo_angle(is_radian=False), "Read joints"))
    print("TCP load:", arm.tcp_load, "| TCP offset:", arm.tcp_offset)


MOVE_KEYS = frozenset(('w', 's', 'a', 'd', 'up', 'down', 'left', 'right'))
ARM_SPEED = 1000.0  # requested mm/s; capped by controller/reduced limits below
COMMAND_DURATION = 0.10  # controller-enforced velocity expiration, seconds
INPUT_TIMEOUT = 0.10  # UI heartbeat expiration, seconds
SEND_INTERVAL = 0.025
GRIPPER_STEP = 5.0  # pulses; no accumulated targets
# Allow the controller's initial "stopped" status to transition to moving.
# Matches the SDK's approximately two-second startup grace period.
GRIPPER_SETTLE = 2.2
GRIPPER_TIMEOUT = 3.0
ZERO = (0.0,) * 6


def speed_limit(arm, reduced=None):
    """Read limits only; never change reduced mode or controller configuration."""
    limits = arm.tcp_speed_limit
    if (not isinstance(limits, (tuple, list)) or len(limits) != 2
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in limits)
            or not 0 <= limits[0] <= limits[1] or limits[1] <= 0):
        raise RuntimeError("Invalid controller speed limit. Teleop disarmed.")
    if reduced is None:
        on, maximum = arm.is_reduced_mode, arm.reduced_max_tcp_speed
    else:
        if not isinstance(reduced, (tuple, list)) or len(reduced) < 3:
            raise RuntimeError("Invalid reduced-mode feedback. Teleop disarmed.")
        on, maximum = reduced[0], reduced[2]
    if type(on) not in (bool, int) or on not in (0, 1):
        raise RuntimeError("Invalid reduced-mode status. Teleop disarmed.")
    speed = min(ARM_SPEED, limits[1])
    if on:
        if type(maximum) not in (int, float) or not math.isfinite(maximum) or maximum <= 0:
            raise RuntimeError("Invalid reduced speed limit. Teleop disarmed.")
        speed = min(speed, maximum)
    return speed


def intent(keys, speed=ARM_SPEED):
    """Opposite keys cancel, diagonal speed is capped, orientation stays fixed."""
    xyz = [float(('w' in keys) - ('s' in keys)),
           float(('a' in keys) - ('d' in keys)),
           float(('up' in keys) - ('down' in keys))]
    length = math.sqrt(sum(v * v for v in xyz))
    velocity = tuple(v * speed / max(1.0, length) for v in xyz) + (0.0,) * 3
    grip = int('left' in keys) - int('right' in keys)
    # Gripper I/O can take longer than a velocity renewal. Never stream arm
    # velocity and gripper movement together.
    if length and grip:
        return ZERO, 0
    return velocity, grip


@dataclass(frozen=True)
class Input:
    keys: frozenset = frozenset()
    focused: bool = False
    updated: float = 0.0


class Cancelled(Exception):
    pass


class Teleop:
    """One worker owns every SDK command; the GUI only publishes input."""
    gripper_speed = 100
    gripper_step = GRIPPER_STEP

    def __init__(self, arm, reports=None, clock=time.monotonic):
        self.arm, self.reports, self.clock = arm, reports, clock
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.input = Input()
        self.armed = self.owned = self.busy = self.stopping = False
        self.processing = False
        self.gripper_ready = False
        self.cancelled = True
        self.closed = False
        self.generation = 0
        self.pending = None
        self.neutral_required = True
        self.velocity = ZERO
        self.speed = ARM_SPEED
        self.last_velocity_send = -math.inf
        self.gripper_target = None
        self.gripper_started = 0.0
        self.gripper_origin = None
        self.gripper_direction = 0
        self.gripper_saw_moving = False
        self.gripper_blocked_direction = 0
        self.pose = self.angles = None
        self.gripper_position = None
        self.message = "DISARMED. Release movement keys, then press E to enable."
        self.next_refresh = 0.0
        self.worker = None

    def publish(self, keys, focused):
        with self.lock:
            keys = frozenset(keys) & MOVE_KEYS
            changed = keys != self.input.keys or bool(focused) != self.input.focused
            self.input = Input(keys, bool(focused), self.clock())
        if changed:
            self.wake.set()

    def current(self):
        with self.lock:
            return self.input

    def require_input(self):
        snapshot = self.current()
        if not snapshot.focused:
            raise RuntimeError("Window lost focus. Press E to enable again.")
        if self.clock() - snapshot.updated > INPUT_TIMEOUT:
            raise RuntimeError("Keyboard updates stopped. Press E to enable again.")
        return snapshot

    def request(self, action):
        with self.lock:
            if self.busy or self.stopping or self.closed or action not in ('enable', 'gripper'):
                return False
            try:
                snapshot = self.require_input()
            except RuntimeError as exc:
                self.message = str(exc)
                return False
            if snapshot.keys:
                self.message = "Release all movement keys before enabling."
                return False
            if action == 'enable':
                if self.armed:
                    return False
                self.cancelled = False  # Only a fresh E press can clear cancellation.
            elif not self.armed or self.gripper_ready:
                self.message = "Press E first." if not self.armed else "Gripper already enabled."
                return False
            self.pending = (action, self.generation)
            self.busy = True
        self.wake.set()
        return True

    def stop(self, reason="Stopped. Press E to enable again."):
        with self.lock:
            self.generation += 1
            self.cancelled = True
            self.armed = self.gripper_ready = False
            self.pending = None
            self.busy = self.processing
            self.neutral_required = True
            self.message = reason
            self.stopping = self.owned
        self.wake.set()

    def valid(self, generation):
        with self.lock:
            if self.cancelled or generation != self.generation or self.closed:
                raise Cancelled()
        self.require_input()

    def send(self, generation, method, *args, claim=False, **kwargs):
        self.valid(generation)
        if claim:
            with self.lock:
                self.valid(generation)
                self.owned = True
        # Never hold the GUI lock across network I/O. A stop arriving during
        # this call is serialized after it by the sole worker; velocity always
        # has a controller-side expiry, including if the host freezes.
        return checked(method(*args, **kwargs), method.__name__)

    def health(self, active=False):
        if not self.arm.connected:
            raise RuntimeError("Robot disconnected. Use the physical E-stop.")
        if self.reports:
            self.reports.check()
        if self.arm.axis != 6:
            raise RuntimeError(f"Expected six axes; received {self.arm.axis}.")
        err, warn = checked(self.arm.get_err_warn_code(), "Read errors")
        if err or warn:
            raise RuntimeError(f"Controller error {err}, warning {warn}. Resolve in Studio.")
        state = checked(self.arm.get_state(), "Read state")
        if active and (self.arm.mode != 5 or state not in (0, 1, 2)):
            raise RuntimeError("Controller state/mode changed. Teleop disarmed.")
        return state

    def enable(self, generation):
        state = self.health()
        if tuple(self.arm.version_number) < (1, 8, 0):
            raise RuntimeError("Firmware 1.8.0 or later is required for timed velocity.")
        if state not in (0, 2, 4):
            raise RuntimeError("Robot is moving or paused. Stop the other controller first.")
        if checked(self.arm.get_cmdnum(), "Read queue") != 0:
            raise RuntimeError("Controller has queued motion. Stop the other program first.")
        reduced = checked(self.arm.get_reduced_states(is_radian=False), "Read speed limits")
        self.speed = min(speed_limit(self.arm, reduced), speed_limit(self.arm))
        self.send(generation, self.arm.set_state, 4, claim=True)
        self.send(generation, self.arm.motion_enable, enable=True)
        self.send(generation, self.arm.set_mode, 5)
        self.send(generation, self.arm.set_state, 0)
        self.send(generation, self.arm.vc_set_cartesian_velocity, list(ZERO),
                  is_radian=False, is_tool_coord=False, duration=COMMAND_DURATION)
        with self.lock:
            self.valid(generation)
            self.armed = True
            self.neutral_required = True
            self.velocity = ZERO
            self.message = "ENABLED. Hold a direction to move; release to stop."

    def read_gripper(self):
        error = checked(self.arm.get_gripper_err_code(), "Read gripper error")
        if error:
            raise RuntimeError(f"Gripper error {error}. Resolve in Studio.")
        pos = checked(self.arm.get_gripper_position(), "Read gripper position")
        if type(pos) not in (int, float) or not math.isfinite(pos) or not 0 <= pos <= 850:
            raise RuntimeError("Invalid gripper position; standard UFACTORY gripper required.")
        status = checked(self.arm.get_gripper_status(), "Read gripper status")
        if type(status) is not int or not 0 <= status <= 65535 or (status & 3) == 3:
            raise RuntimeError("Invalid gripper motion status. Teleop disarmed.")
        self.gripper_position = pos
        return pos, status & 3

    def enable_gripper(self, generation):
        self.health(active=True)
        self.zero_velocity(generation)
        version = checked(self.arm.get_gripper_version(), "Read gripper firmware")
        try:
            version = tuple(int(part) for part in version.split('.'))
        except (ValueError, AttributeError):
            version = ()
        if len(version) != 3 or version < (3, 4, 3):
            raise RuntimeError("Gripper firmware 3.4.3 or later is required for motion feedback.")
        _, status = self.read_gripper()
        if status == 1:
            raise RuntimeError("Gripper is still moving. Wait for it to stop before enabling.")
        self.send(generation, self.arm.set_gripper_mode, 0)
        self.send(generation, self.arm.set_gripper_enable, True)
        self.send(generation, self.arm.set_gripper_speed, self.gripper_speed)
        with self.lock:
            self.valid(generation)
            self.gripper_ready = True
            # A previous session's target is obsolete after verified stopped
            # feedback. In particular, it must not cause another false timeout.
            self.gripper_target = None
            self.gripper_blocked_direction = 0
            self.gripper_saw_moving = False
            self.neutral_required = True
            self.message = "Gripper enabled. Hold LEFT to open; RIGHT to close."

    def zero_velocity(self, generation):
        if self.velocity != ZERO:
            self.send(generation, self.arm.vc_set_cartesian_velocity, list(ZERO),
                      is_radian=False, is_tool_coord=False, duration=COMMAND_DURATION)
            self.velocity = ZERO

    def grip(self, generation, direction):
        if not self.gripper_ready:
            self.message = "Release the arrow, press G once, then hold LEFT or RIGHT."
            return
        pos, status = self.read_gripper()
        if self.gripper_target is not None:
            elapsed = self.clock() - self.gripper_started
            progress = (pos - self.gripper_origin) * self.gripper_direction
            reached = abs(pos - self.gripper_target) <= 1
            if status == 1:
                self.gripper_saw_moving = True
                if elapsed > GRIPPER_TIMEOUT:
                    raise RuntimeError("Gripper still reports motion after 3 seconds. Teleop stopped.")
                return
            if not (reached or progress > 0 or self.gripper_saw_moving
                    or elapsed >= GRIPPER_SETTLE):
                # A read just after sending may still show the old stopped state.
                return
            if (status == 2 and self.gripper_direction < 0) or (not reached and progress <= 0):
                self.gripper_blocked_direction = self.gripper_direction
            self.gripper_target = None
        elif status == 1:
            raise RuntimeError("Unexpected gripper motion. Teleop stopped.")
        # Re-read input after blocking gripper I/O; release cancels the next step.
        self.valid(generation)
        velocity, latest = intent(self.require_input().keys)
        if velocity != ZERO or latest != direction:
            return
        if direction == self.gripper_blocked_direction or (status == 2 and direction < 0):
            self.message = "Gripper stopped / grasped. Reverse direction to move away."
            return
        target = max(0.0, min(850.0, pos + direction * self.gripper_step))
        if target == pos:
            self.message = "Gripper travel limit reached. Reverse direction to move away."
            return
        self.send(generation, self.arm.set_gripper_position, target, wait=False,
                  wait_motion=False, auto_enable=False)
        self.gripper_target, self.gripper_started = target, self.clock()
        self.gripper_origin, self.gripper_direction = pos, direction
        self.gripper_saw_moving = False
        self.gripper_blocked_direction = 0
        self.message = "Opening gripper…" if direction > 0 else "Closing gripper…"

    def refresh(self):
        self.pose = vector(checked(self.arm.get_position(is_radian=False), "Read pose"), "pose")
        self.angles = vector(checked(self.arm.get_servo_angle(is_radian=False), "Read joints"),
                             "joints", allow_seventh=True)
        self.next_refresh = self.clock() + 0.5

    def _stop_now(self):
        if not self.stopping:
            return
        try:
            # Do not disable gripping force: a held object could drop. The
            # gripper's already-issued target is at most one bounded increment.
            checked(self.arm.set_state(4), "Arm stop")
            with self.lock:
                self.owned = False
                self.message += " [Stop acknowledged]"
        except Exception as exc:
            self.message = f"STOP NOT CONFIRMED: {exc}. Use physical E-stop."
        finally:
            self.velocity = ZERO
            self.stopping = False
            self.busy = False

    def tick(self):
        """One control cycle, also exercised by offline tests."""
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
                if self.clock() >= self.next_refresh:
                    self.refresh()
                return
            self.require_input()
            self.health(active=True)
            self.valid(generation)
            snapshot = self.require_input()
            if self.neutral_required:
                if snapshot.keys:
                    self.message = "Release movement keys once before moving."
                    return
                self.neutral_required = False
            # Rich reports refresh these properties without blocking velocity
            # renewal. Reductions take effect immediately; speed never rises
            # above the limit accepted at E-enable until the next fresh enable.
            self.speed = min(self.speed, speed_limit(self.arm))
            velocity, grip = intent(snapshot.keys, self.speed)
            if not grip:
                self.gripper_blocked_direction = 0
            if velocity != ZERO:
                if velocity != self.velocity or self.clock() - self.last_velocity_send >= SEND_INTERVAL:
                    # No long-lived target or motion queue: every command expires.
                    self.send(generation, self.arm.vc_set_cartesian_velocity, list(velocity),
                              is_radian=False, is_tool_coord=False, duration=COMMAND_DURATION)
                    self.velocity, self.last_velocity_send = velocity, self.clock()
                    self.message = "Moving while held. Release to stop."
            else:
                self.zero_velocity(generation)
                if grip:
                    self.grip(generation, grip)
                elif snapshot.keys and any(k in snapshot.keys for k in ('left', 'right')):
                    self.message = "Use arm directions or the gripper separately."
                else:
                    self.message = "Ready. Hold a direction; release to stop."
            # Read feedback only while the arm is stationary: reads must never
            # delay a required velocity renewal or key-release zero command.
            if self.velocity == ZERO and not grip and self.clock() >= self.next_refresh:
                self.refresh()
        except Cancelled:
            pass
        except Exception as exc:
            self.stop(str(exc))
        finally:
            self._stop_now()

    def start(self):
        def run():
            while True:
                self.wake.clear()
                self.tick()
                if self.closed:
                    return
                self.wake.wait(SEND_INTERVAL)
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()

    def close(self):
        self.stop("Controller exiting.")
        self.closed = True
        self.wake.set()
        if self.worker:
            self.worker.join(4)
            if self.worker.is_alive():
                self.message = "Shutdown timed out. Use physical E-stop."
        else:
            self._stop_now()


class Keyboard:
    """Debounced enable/stop controls, true held-state input, focus interlock."""
    def __init__(self):
        self.previous = frozenset()

    def frame(self, controller, held, focused, focus_lost=False, quit_requested=False):
        held = frozenset(held)
        fresh = held - self.previous
        self.previous = held
        controller.publish(held & MOVE_KEYS, focused)
        if quit_requested or 'escape' in held or 'q' in held:
            controller.stop("Quit requested.")
            return False
        # A transient focus loss disarms even if focus returns in the same frame.
        if focus_lost or not focused:
            if controller.armed or controller.busy or controller.owned:
                controller.stop("Window lost focus. Press E to enable again.")
            return True
        if 'space' in held:
            if 'space' in fresh or controller.armed or controller.busy:
                controller.stop()
            return True
        if 'e' in fresh:
            controller.request('enable')
        elif 'g' in fresh:
            controller.request('gripper')
        return True


def draw_window(pg, surface, controller, ip, fonts, held):
    bg, panel, text, muted = '#10181e', '#1c2932', '#edf4f5', '#a1b4c0'
    accent = '#68d6b4' if controller.armed else '#efbd69'
    surface.fill(bg)
    title, normal, small = fonts

    def label(value, x, y, font=normal, color=text):
        surface.blit(font.render(value, True, color), (x, y))

    label("xArm 6 · Hold to move", 30, 22, title)
    label(ip + f"  ·  Base coordinates  ·  Up to {controller.speed:,.0f} mm/s",
          30, 65, small, '#efbd69')
    pg.draw.rect(surface, panel, (30, 100, 760, 60), border_radius=12)
    label("ENABLED" if controller.armed else "DISARMED", 48, 116, normal, accent)
    label("E  Enable arm     G  Enable gripper", 280, 118, small)

    cards = [('W', '+X', 'w'), ('S', '−X', 's'), ('A', '+Y', 'a'), ('D', '−Y', 'd'),
             ('↑', '+Z', 'up'), ('↓', '−Z', 'down'),
             ('←', 'Open gripper', 'left'), ('→', 'Close gripper', 'right')]
    for i, (key, action, name) in enumerate(cards):
        x, y = 30 + (i % 4) * 192, 185 + (i // 4) * 105
        active = name in held and controller.armed
        pg.draw.rect(surface, '#245b50' if active else panel, (x, y, 184, 92), border_radius=12)
        label(key, x + 16, y + 10, title, '#68d6b4' if active else text)
        label(action, x + 16, y + 55, small, muted)
    label("Hold to move. Release to stop. No Enter needed.", 30, 409)
    label("SPACE  Stop / disarm       ESC or Q  Quit", 30, 445, small, '#efbd69')
    label("Changing windows disarms the controls. Keep the physical E-stop in reach.",
          30, 474, small, muted)
    label("Gripper release stops new steps; its last 5-pulse step may finish.",
          30, 500, small, muted)
    message = ("WORKING: " if controller.busy else "") + controller.message
    # Wrap status/error text, including long SDK errors.
    words, line, lines = message.split(), "", []
    for word in words:
        candidate = (line + " " + word).strip()
        if small.size(candidate)[0] > 750:
            lines.append(line)
            line = word
        else:
            line = candidate
    lines.append(line)
    for index, line in enumerate(lines[:3]):
        label(line, 30, 544 + index * 23, small, accent)


def window(controller, ip, max_frames=None):
    import pygame as pg
    pg.display.init()
    pg.font.init()
    try:
        surface = pg.display.set_mode((820, 630))
        pg.display.set_caption("xArm 6 — Hold-to-move controls")
        pg.key.set_repeat()  # Held-state polling, never operating-system autorepeat.
        fonts = [pg.font.SysFont('Arial', size) for size in (30, 21, 17)]
        keys = {pg.K_w: 'w', pg.K_s: 's', pg.K_a: 'a', pg.K_d: 'd',
                pg.K_UP: 'up', pg.K_DOWN: 'down', pg.K_LEFT: 'left', pg.K_RIGHT: 'right',
                pg.K_e: 'e', pg.K_g: 'g', pg.K_SPACE: 'space', pg.K_ESCAPE: 'escape',
                pg.K_q: 'q'}
        keyboard = Keyboard()
        timer = pg.time.Clock()
        frames = 0
        controller.start()
        while max_frames is None or frames < max_frames:
            events = pg.event.get()  # Pumps KEYDOWN/KEYUP before reading held state.
            pressed = pg.key.get_pressed()
            held = {name for key, name in keys.items() if pressed[key]}
            # Space/Escape/Q must work even if pressed and released between frames.
            for event in events:
                if event.type == pg.KEYDOWN and event.key in (
                        pg.K_SPACE, pg.K_ESCAPE, pg.K_q, pg.K_e, pg.K_g):
                    held.add(keys[event.key])
            lost = any(e.type in (pg.WINDOWFOCUSLOST, pg.WINDOWMINIMIZED) for e in events)
            quit_requested = any(e.type == pg.QUIT for e in events)
            if not keyboard.frame(controller, held, pg.key.get_focused(), lost, quit_requested):
                break
            draw_window(pg, surface, controller, ip, fonts, held)
            pg.display.flip()
            timer.tick(60)
            frames += 1
    finally:
        controller.close()
        pg.quit()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default='192.168.2.205',
                        type=lambda value: str(ipaddress.IPv4Address(value)))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-port', action='store_true', help='TCP connection only; no SDK')
    mode.add_argument('--status', action='store_true', help='Read status only; never enable/move')
    args = parser.parse_args(argv)
    if not probe(args.ip):
        return 2
    if args.check_port:
        return 0
    arm = controller = control_lock = None
    previous_signals = {}
    try:
        if not args.status:
            control_lock = acquire_control_lock()
        arm, reports = prepare_arm(args.ip)
        arm.connect()
        arm.set_timeout(0.5)
        if not arm.connected:
            raise RuntimeError("SDK connection failed")
        if not reports.received.wait(3):
            raise RuntimeError("No controller status reports received")
        reports.check()
        read_status(arm)
        if args.status:
            return 0
        controller = Teleop(arm, reports)
        controller.health()

        def interrupted(*_):
            raise KeyboardInterrupt

        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGTSTP):
            previous_signals[sig] = signal.getsignal(sig)
            # Suspending the GUI must not leave a held key active.
            signal.signal(sig, interrupted)
        window(controller, args.ip)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Teleop stopped: {exc}")
        return 1
    finally:
        if controller and not controller.closed:
            controller.close()
        if controller:
            print(controller.message)
        if arm:
            arm.disconnect()
        if control_lock:
            control_lock.close()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    sys.exit(main())
