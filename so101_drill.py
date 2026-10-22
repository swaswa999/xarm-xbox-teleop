"""Optional STS3215 drill at ID 8; the SO-101 joints may share its USB bus.

The serial worker is independent of xArm SDK I/O. No serial port opens until
the operator enables control. A process/host crash cannot guarantee a stop:
this servo's velocity mode has no command expiry. Keep power reachable.
"""
import threading
import time

DRILL_BUTTONS = frozenset({'dleft', 'dright'})
MAX_SPEED = 2000  # Existing controller limit, not a measured shaft speed.


def direction(pad):
    return int('dright' in pad.buttons) - int('dleft' in pad.buttons)


class SerialDrill:
    def __init__(self, port, motor_id=8):
        if motor_id != 8:
            raise ValueError('This drill profile requires the verified motor ID 8.')
        self.path, self.motor_id = port, motor_id
        self.port = self.packet = None
        self.identified = self.claimed = False

    def open(self):
        import scservo_sdk as sdk
        self.port = sdk.PortHandler(self.path)
        self.packet = sdk.PacketHandler(0)
        if not self.port.openPort():
            raise RuntimeError('Cannot open the drill USB controller.')
        # PortHandler defaults to 1 Mbaud. Do not reopen after taking exclusivity.
        self.port.ser.write_timeout = .1
        if self.port.getBaudRate() != 1000000:
            raise RuntimeError('Drill bus must use 1000000 baud.')
        import fcntl
        import termios
        # Kernel exclusivity also blocks the old browser motor panel from
        # opening this device, unlike a cooperating file lock alone.
        fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
        model, comm, error = self.packet.ping(self.port, self.motor_id)
        if (model, comm, error) != (777, 0, 0):
            raise RuntimeError(f'No healthy STS3215 at ID 8 on {self.path}.')
        self.identified = True
        # IDs 1–6 may be connected on the same bus. All reads and writes below
        # are unicast to ID 8; never configure, disable, or broadcast to joints.

    def read(self, address, size=1):
        fn = self.packet.read1ByteTxRx if size == 1 else self.packet.read2ByteTxRx
        value, comm, error = fn(self.port, self.motor_id, address)
        if comm or error:
            raise RuntimeError(f'Drill read {address} failed: {comm}, {error}.')
        return value

    def write(self, address, value, size=1):
        fn = self.packet.write1ByteTxRx if size == 1 else self.packet.write2ByteTxRx
        comm, error = fn(self.port, self.motor_id, address, value)
        if comm or error:
            raise RuntimeError(f'Drill write {address} failed: {comm}, {error}.')

    def prepare(self, valid):
        valid()
        self.claimed = True
        self.write(40, 0)
        if self.read(33) != 1:
            self.write(55, 0)
            try:
                valid()
                self.write(33, 1)
            finally:
                self.write(55, 1)
        if self.read(33) != 1:
            raise RuntimeError('Drill speed mode did not verify.')
        self.write(41, 100)
        self.write(48, 1000, 2)
        self.velocity(0)
        valid()  # Stop/focus loss during setup must prevent torque enable.
        self.write(40, 1)
        valid()

    def velocity(self, speed):
        if type(speed) is not int or abs(speed) > MAX_SPEED:
            raise ValueError('Invalid drill speed.')
        self.write(46, abs(speed) | (32768 if speed < 0 else 0), 2)

    def health(self):
        if self.read(40) != 1:
            raise RuntimeError('Drill torque was disabled or protection tripped.')

    def close(self):
        errors = []
        if self.port and self.port.is_open:
            try:
                if self.claimed:
                    # Attempt torque off even when the zero-speed write fails.
                    try:
                        self.velocity(0)
                    except Exception as exc:
                        errors.append(str(exc))
                    try:
                        self.write(40, 0)
                        if self.read(40) != 0:
                            raise RuntimeError('Drill torque off did not verify.')
                    except Exception as exc:
                        errors.append(str(exc))
            finally:
                self.port.closePort()
        self.claimed = False
        if errors:
            raise RuntimeError('Drill stop unconfirmed: ' + '; '.join(errors))


class DrillController:
    """One serial owner; UI only publishes snapshots and cancellation tokens."""
    def __init__(self, factory, enabled, on_fault, clock=time.monotonic):
        self.factory, self.enabled, self.on_fault, self.clock = factory, enabled, on_fault, clock
        self.lock = threading.Lock()
        self.pad = None
        self.updated = 0.
        self.generation = 0
        self.handled_generation = 0
        self.closed = False
        self.hardware = None
        self.worker = None
        self.wake = threading.Event()
        self.speed = 0
        self.ready = False
        self.must_release = True
        self.reverse_after = 0.
        self.next_health = 0.
        self.message = 'Drill OFF · Menu enables arm + drill'

    def publish(self, pad):
        with self.lock:
            self.pad, self.updated = pad, self.clock()
        self.wake.set()

    def disarm(self):
        with self.lock:
            self.generation += 1
        self.wake.set()

    def valid(self, generation):
        with self.lock:
            pad, updated = self.pad, self.updated
            cancelled = self.closed or generation != self.generation
        now = self.clock()
        if cancelled or not self.enabled():
            raise RuntimeError('Drill disarmed.')
        if (pad is None or not pad.connected or not pad.focused
                or not 0 <= now - updated <= .10
                or not 0 <= now - pad.updated <= .15
                or pad.buttons & {'view', 'space', 'escape', 'q'}):
            raise RuntimeError('Drill input lost or stopped. Press Menu to enable again.')
        return pad

    def shutdown_motor(self):
        hardware, self.hardware = self.hardware, None
        self.ready = False
        self.speed = 0
        self.must_release = True
        self.reverse_after = 0.
        if hardware:
            hardware.close()

    def tick(self):
        with self.lock:
            generation = self.generation
        try:
            if self.closed or not self.enabled() or generation != self.handled_generation:
                was_ready = self.ready
                self.shutdown_motor()
                self.handled_generation = generation
                if was_ready:
                    self.message = 'Drill OFF · Menu enables arm + drill'
                return
            pad = self.valid(generation)
            if self.hardware is None:
                if not pad.neutral:
                    self.message = 'Drill: release all controls before enabling'
                    return
                self.hardware = self.factory()
                self.hardware.open()
                self.hardware.prepare(lambda: self.valid(generation))
                self.ready = True
                self.must_release = True
                self.message = 'Drill ready · Hold D-pad right / left'
                self.next_health = self.clock()
                return
            if self.clock() >= self.next_health:
                self.hardware.health()
                self.next_health = self.clock() + .10
            if self.must_release:
                if pad.buttons & DRILL_BUTTONS:
                    return
                self.must_release = False
            requested = direction(pad) * MAX_SPEED
            # Reversal first sends zero and waits on later ticks, so Stop and
            # controller watchdogs remain responsive during the pause.
            if requested * self.speed < 0:
                self.hardware.velocity(0)
                self.speed = 0
                self.reverse_after = self.clock() + .15
                self.message = 'Drill stopping before reversal'
                return
            if self.clock() < self.reverse_after:
                return
            latest = self.valid(generation)
            if direction(latest) * MAX_SPEED != requested:
                return
            if requested != self.speed:
                self.hardware.velocity(requested)
                self.speed = requested
            self.message = ('Drill FORWARD · 2000' if self.speed > 0 else
                            'Drill REVERSE · 2000' if self.speed < 0 else
                            'Drill ready · Hold D-pad right / left')
        except Exception as exc:
            message = str(exc)
            try:
                self.shutdown_motor()
            except Exception as stop_exc:
                message += ' ' + str(stop_exc)
            self.message = message
            self.on_fault(message)

    def start(self):
        def run():
            while not self.closed:
                self.wake.clear()
                self.tick()
                self.wake.wait(.02)
            self.tick()
        self.worker = threading.Thread(target=run, daemon=True, name='drill-owner')
        self.worker.start()

    def close(self):
        self.closed = True
        self.disarm()
        if self.worker:
            self.worker.join(2)
            if self.worker.is_alive():
                self.message = 'Drill shutdown timed out; switch off motor power.'
                self.on_fault(self.message)
        else:
            self.tick()
