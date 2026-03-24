#!/usr/bin/env python
"""
GPIO Stepper Motor Driver for KORUZA
Controls three 28BYJ-48 stepper motors via ULN2003 driver boards.
Drop-in replacement for the original I2C Motor class.

GPIO pin assignments (BCM):
  X axis (pan):   IN1=5,  IN2=6,  IN3=13, IN4=26  (physical 29,31,33,37)
  Y axis (tilt):  IN1=12, IN2=16, IN3=20, IN4=21  (physical 32,36,38,40)
  F axis (focus): IN1=17, IN2=27, IN3=22, IN4=23  (physical 11,13,15,16)

These pins avoid I2C (2,3), UART (14,15), primary SPI (7-11), and 1-Wire (4).
"""

import RPi.GPIO as GPIO
import threading
import time
import logging

logger = logging.getLogger(__name__)


class GPIOStepperMotor(object):
    """
    GPIO-based stepper motor controller for three 28BYJ-48 motors (X, Y, F axes).

    Hardware:
    - 28BYJ-48 5-wire unipolar stepper motors
    - ULN2003 driver boards (4 control pins per motor)
    - Half-step mode: 8 steps per electrical cycle, 4096 steps per revolution

    Interface (compatible with original I2C Motor class):
    - read()                          -> status dict, NEVER None
    - move(x, y, f)                   -> non-blocking move to target positions
    - configure(command, laser, speed, accel)
    - serialize()                     -> metadata dict
    - cleanup()                       -> release GPIO
    """

    # Half-step sequence for 28BYJ-48 (8 steps, smoother than full-step)
    HALF_STEP_SEQUENCE = [
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1],
        [1, 0, 0, 1],
    ]

    # Status constants (match I2C driver)
    STATUS_IDLE   = 0
    STATUS_MOVING = 1
    STATUS_HOMING = 2

    # Motor command constants (match I2C driver)
    CMD_NONE   = 0
    CMD_STOP   = 1
    CMD_HOME_X = 2
    CMD_HOME_Y = 3
    CMD_HOME_F = 4

    def __init__(self, pins_x, pins_y, pins_f,
                 max_x=35000, max_y=35000, max_f=20000,
                 steps_per_unit=1.0, step_delay=0.002):
        """
        Args:
            pins_x/y/f:     Tuple of 4 BCM GPIO pins (IN1, IN2, IN3, IN4)
            max_x/y/f:      Maximum position in KORUZA units
            steps_per_unit: Motor steps per KORUZA position unit
            step_delay:     Seconds between steps (controls speed)
        """
        self.pins = {
            'x': list(pins_x),
            'y': list(pins_y),
            'f': list(pins_f),
        }
        self.max_positions = {'x': max_x, 'y': max_y, 'f': max_f}
        self.steps_per_unit = steps_per_unit
        # 28BYJ-48 reliable maximum is ~500 steps/sec (2 ms/step) under load.
        # Enforce a hard floor so the motor never skips steps silently.
        self.step_delay = max(0.002, step_delay)

        # Positions in KORUZA units (in-memory only, reset to 0 on restart)
        self.current_position = {'x': 0, 'y': 0, 'f': 0}
        self.target_position  = {'x': 0, 'y': 0, 'f': 0}

        # Stepper sequence index per axis (0-7)
        self.step_index = {'x': 0, 'y': 0, 'f': 0}

        # Per-axis status
        self.status = {
            'x': self.STATUS_IDLE,
            'y': self.STATUS_IDLE,
            'f': self.STATUS_IDLE,
        }

        # Movement thread state
        self.move_thread = None
        self.stop_flag   = False
        self.lock        = threading.Lock()

        # Configurable parameters (exposed via read())
        self.laser_enabled  = False
        self.speed          = 1000   # KORUZA units/sec (used to compute step_delay)
        self.accel          = 1000   # stored for I2C compatibility, not used for motion
        self._last_command  = self.CMD_NONE

        self._init_gpio()
        logger.info("GPIOStepperMotor ready: X=%s Y=%s F=%s", pins_x, pins_y, pins_f)

    # ------------------------------------------------------------------
    # GPIO helpers
    # ------------------------------------------------------------------

    def _init_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for axis in ('x', 'y', 'f'):
            for pin in self.pins[axis]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, 0)
        logger.info("GPIO pins initialised")

    def _set_step(self, axis, pattern):
        for i, pin in enumerate(self.pins[axis]):
            GPIO.output(pin, pattern[i])

    def _motor_off(self, axis):
        """De-energise all coils for one axis (prevents overheating)."""
        for pin in self.pins[axis]:
            GPIO.output(pin, 0)

    def _step_motor(self, axis, direction):
        """Advance one step in direction (+1 forward, -1 backward)."""
        self.step_index[axis] = (self.step_index[axis] + direction) % 8
        self._set_step(axis, self.HALF_STEP_SEQUENCE[self.step_index[axis]])

    # ------------------------------------------------------------------
    # Movement implementation
    # ------------------------------------------------------------------

    def _move_axis(self, axis, target_units):
        """
        Move one axis to target_units. Blocks until done or stop_flag set.
        Updates current_position to reflect actual steps completed.
        """
        target_units = int(max(0, min(target_units, self.max_positions[axis])))

        with self.lock:
            current = self.current_position[axis]

        steps_to_move = int((target_units - current) * self.steps_per_unit)
        if steps_to_move == 0:
            return

        direction = 1 if steps_to_move > 0 else -1
        abs_steps = abs(steps_to_move)
        completed = 0

        for _ in range(abs_steps):
            if self.stop_flag:
                break
            self._step_motor(axis, direction)
            time.sleep(self.step_delay)
            completed += 1

        # Update position based on steps actually completed (handles mid-stop correctly)
        if self.steps_per_unit > 0:
            units_moved = int(completed / self.steps_per_unit)
        else:
            units_moved = 0

        new_pos = max(0, min(current + direction * units_moved, self.max_positions[axis]))
        with self.lock:
            self.current_position[axis] = new_pos

    def _move_thread_func(self):
        """Background thread: move all axes then de-energise coils."""
        try:
            with self.lock:
                for axis in ('x', 'y', 'f'):
                    if self.target_position[axis] != self.current_position[axis]:
                        self.status[axis] = self.STATUS_MOVING

            for axis in ('x', 'y', 'f'):
                if self.stop_flag:
                    break
                self._move_axis(axis, self.target_position[axis])

            # De-energise all coils
            for axis in ('x', 'y', 'f'):
                self._motor_off(axis)
                self.status[axis] = self.STATUS_IDLE

            # If stopped early, reset targets to current so UI shows consistent state
            if self.stop_flag:
                with self.lock:
                    for axis in ('x', 'y', 'f'):
                        self.target_position[axis] = self.current_position[axis]

        except Exception as e:
            logger.error("Move thread error: %s", e)
        finally:
            self.move_thread = None

    # ------------------------------------------------------------------
    # Public interface (I2C Motor class compatible)
    # ------------------------------------------------------------------

    def read(self):
        """
        Return current motor status. NEVER returns None.

        Keys match the original I2C Motor.read() output exactly:
          current_x/y/f, next_x/y/f, status_x/y/f,
          flash_status, flash_write_count, command, laser, speed, accel, empty
        """
        with self.lock:
            return {
                'current_x':        self.current_position['x'],
                'current_y':        self.current_position['y'],
                'current_f':        self.current_position['f'],
                'next_x':           self.target_position['x'],
                'next_y':           self.target_position['y'],
                'next_f':           self.target_position['f'],
                'status_x':         self.status['x'],
                'status_y':         self.status['y'],
                'status_f':         self.status['f'],
                'flash_status':     0,
                'flash_write_count': 0,
                'command':          self._last_command,
                'laser':            1 if self.laser_enabled else 0,
                'speed':            self.speed,
                'accel':            self.accel,
                'empty':            255,   # I2C validity marker — always 255
            }

    def move(self, x, y, f):
        """
        Move motors to target positions (non-blocking).

        Args:
            x, y, f: Target positions in KORUZA units (keyword or positional)
        """
        # Stop any in-progress movement
        if self.move_thread and self.move_thread.is_alive():
            self.stop_flag = True
            self.move_thread.join(timeout=2.0)

        self.stop_flag = False

        with self.lock:
            self.target_position['x'] = int(max(0, min(x, self.max_positions['x'])))
            self.target_position['y'] = int(max(0, min(y, self.max_positions['y'])))
            self.target_position['f'] = int(max(0, min(f, self.max_positions['f'])))

        self.move_thread = threading.Thread(target=self._move_thread_func)
        self.move_thread.daemon = True
        self.move_thread.start()

        logger.info("move: X=%d Y=%d F=%d", x, y, f)
        return True

    def configure(self, command=None, laser=None, speed=None, accel=None):
        """
        Configure motor parameters.

        command values:
            0 / None = no-op
            1        = STOP — halt current movement
            2        = HOME X — reset X position counter to 0 at current location
            3        = HOME Y — reset Y position counter to 0 at current location
            4        = HOME F — reset F position counter to 0 at current location

        Note: no physical limit switches, so HOME resets the software position
        counter rather than driving to a hardware endstop.
        """
        if command is not None:
            self._last_command = int(command)

            if command == self.CMD_STOP:
                self.stop_flag = True
                logger.info("configure: STOP")

            elif command == self.CMD_HOME_X:
                with self.lock:
                    self.current_position['x'] = 0
                    self.target_position['x']  = 0
                    self.status['x']           = self.STATUS_IDLE
                logger.info("configure: HOME X")

            elif command == self.CMD_HOME_Y:
                with self.lock:
                    self.current_position['y'] = 0
                    self.target_position['y']  = 0
                    self.status['y']           = self.STATUS_IDLE
                logger.info("configure: HOME Y")

            elif command == self.CMD_HOME_F:
                with self.lock:
                    self.current_position['f'] = 0
                    self.target_position['f']  = 0
                    self.status['f']           = self.STATUS_IDLE
                logger.info("configure: HOME F")

        if laser is not None:
            self.laser_enabled = bool(laser)
            logger.info("configure: laser=%s", self.laser_enabled)

        if speed is not None:
            self.speed = int(speed)
            # Recalculate step delay, but never go faster than 500 steps/sec
            # (2 ms floor) — the 28BYJ-48 skips steps silently above this under load.
            if self.speed > 0 and self.steps_per_unit > 0:
                self.step_delay = max(0.002, 1.0 / (self.speed * self.steps_per_unit))
            logger.info("configure: speed=%d step_delay=%.4fs", self.speed, self.step_delay)

        if accel is not None:
            self.accel = int(accel)
            logger.info("configure: accel=%d", self.accel)

        return True

    def serialize(self):
        """Return metadata dict describing this motor driver instance."""
        return {
            'type':   'gpio_stepper',
            'pins_x': self.pins['x'],
            'pins_y': self.pins['y'],
            'pins_f': self.pins['f'],
        }

    def cleanup(self):
        """Stop movement and release GPIO resources."""
        if self.move_thread and self.move_thread.is_alive():
            self.stop_flag = True
            self.move_thread.join(timeout=2.0)

        for axis in ('x', 'y', 'f'):
            self._motor_off(axis)

        logger.info("GPIOStepperMotor cleanup done")

    def __del__(self):
        try:
            self.cleanup()
        except Exception:
            pass
