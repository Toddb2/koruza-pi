#!/usr/bin/env python
"""
GPIO Stepper Motor Driver for KORUZA
Controls three 28BYJ-48 stepper motors via ULN2003 driver boards
Provides same interface as I2C Motor class for drop-in replacement
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
    - Half-step mode: 4096 steps per revolution

    Interface compatibility with I2C Motor class:
    - read(): Returns current motor status
    - move(x, y, f): Move motors to target positions
    - configure(laser, speed, accel): Configure motor parameters
    """

    # Half-step sequence for 28BYJ-48 (smoother than full-step)
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

    # Motor status constants (match I2C driver)
    STATUS_IDLE = 0
    STATUS_MOVING = 1
    STATUS_HOMING = 2

    def __init__(self, pins_x, pins_y, pins_f,
                 max_x=35000, max_y=35000, max_f=20000,
                 steps_per_unit=1.0, step_delay=0.002):
        """
        Initialize GPIO stepper motor controller.

        Args:
            pins_x: Tuple of 4 GPIO pins for X motor (IN1, IN2, IN3, IN4)
            pins_y: Tuple of 4 GPIO pins for Y motor (IN1, IN2, IN3, IN4)
            pins_f: Tuple of 4 GPIO pins for F motor (IN1, IN2, IN3, IN4)
            max_x: Maximum position for X axis in KORUZA units
            max_y: Maximum position for Y axis in KORUZA units
            max_f: Maximum position for F axis in KORUZA units
            steps_per_unit: Motor steps per KORUZA position unit
            step_delay: Delay between steps in seconds (controls speed)
        """
        self.pins = {
            'x': list(pins_x),
            'y': list(pins_y),
            'f': list(pins_f)
        }

        self.max_positions = {
            'x': max_x,
            'y': max_y,
            'f': max_f
        }

        self.steps_per_unit = steps_per_unit
        self.step_delay = step_delay

        # Current positions in KORUZA units
        self.current_position = {'x': 0, 'y': 0, 'f': 0}
        self.target_position = {'x': 0, 'y': 0, 'f': 0}

        # Current step index in sequence (0-7)
        self.step_index = {'x': 0, 'y': 0, 'f': 0}

        # Motor status
        self.status = {'x': self.STATUS_IDLE, 'y': self.STATUS_IDLE, 'f': self.STATUS_IDLE}

        # Movement thread
        self.move_thread = None
        self.stop_flag = False
        self.lock = threading.Lock()

        # Motor parameters
        self.laser_enabled = False
        self.speed = 1000  # units/sec (not used in simple implementation)
        self.accel = 1000  # units/sec^2 (not used in simple implementation)

        # Initialize GPIO
        self._init_gpio()

        logger.info("GPIO Stepper Motor initialized: X=%s, Y=%s, F=%s", pins_x, pins_y, pins_f)

    def _init_gpio(self):
        """Initialize GPIO pins."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Set up all motor pins as outputs
        for axis in ['x', 'y', 'f']:
            for pin in self.pins[axis]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, 0)

        logger.info("GPIO pins initialized")

    def _set_step(self, axis, step_pattern):
        """
        Set GPIO outputs for one step.

        Args:
            axis: 'x', 'y', or 'f'
            step_pattern: List of 4 values [IN1, IN2, IN3, IN4]
        """
        for i, pin in enumerate(self.pins[axis]):
            GPIO.output(pin, step_pattern[i])

    def _motor_off(self, axis):
        """Turn off all coils for given axis to save power."""
        for pin in self.pins[axis]:
            GPIO.output(pin, 0)

    def _step_motor(self, axis, direction):
        """
        Move motor one step in given direction.

        Args:
            axis: 'x', 'y', or 'f'
            direction: 1 for forward, -1 for backward
        """
        # Update step index
        self.step_index[axis] = (self.step_index[axis] + direction) % 8

        # Set coil pattern
        self._set_step(axis, self.HALF_STEP_SEQUENCE[self.step_index[axis]])

    def _move_axis(self, axis, target_units):
        """
        Move single axis to target position.

        Args:
            axis: 'x', 'y', or 'f'
            target_units: Target position in KORUZA units
        """
        # Clamp target to valid range
        target_units = max(0, min(target_units, self.max_positions[axis]))

        current = self.current_position[axis]
        steps_to_move = int((target_units - current) * self.steps_per_unit)

        if steps_to_move == 0:
            return

        direction = 1 if steps_to_move > 0 else -1
        steps_to_move = abs(steps_to_move)

        # Move motor
        for _ in range(steps_to_move):
            if self.stop_flag:
                break

            self._step_motor(axis, direction)
            time.sleep(self.step_delay)

        # Update position
        with self.lock:
            self.current_position[axis] = target_units

    def _move_thread_func(self):
        """Thread function for non-blocking motor movement."""
        try:
            # Set all axes to moving
            for axis in ['x', 'y', 'f']:
                if self.target_position[axis] != self.current_position[axis]:
                    self.status[axis] = self.STATUS_MOVING

            # Move all axes (simplified: move sequentially, could be optimized for simultaneous)
            for axis in ['x', 'y', 'f']:
                if self.stop_flag:
                    break
                self._move_axis(axis, self.target_position[axis])

            # Turn off motors to save power
            for axis in ['x', 'y', 'f']:
                self._motor_off(axis)
                self.status[axis] = self.STATUS_IDLE

        except Exception as e:
            logger.error("Error in move thread: %s", e)
        finally:
            self.move_thread = None

    def read(self):
        """
        Read current motor status (interface compatible with I2C Motor class).

        Returns:
            dict: Motor status with keys:
                - current_x, current_y, current_f: Current positions
                - next_x, next_y, next_f: Target positions
                - status_x, status_y, status_f: Motor status (0=idle, 1=moving)
                - flash_status: Always 0 (not applicable)
                - laser: Laser enable flag
        """
        with self.lock:
            return {
                'current_x': self.current_position['x'],
                'current_y': self.current_position['y'],
                'current_f': self.current_position['f'],
                'next_x': self.target_position['x'],
                'next_y': self.target_position['y'],
                'next_f': self.target_position['f'],
                'status_x': self.status['x'],
                'status_y': self.status['y'],
                'status_f': self.status['f'],
                'flash_status': 0,
                'laser': 1 if self.laser_enabled else 0
            }

    def move(self, x, y, f):
        """
        Move motors to target positions (non-blocking).

        Args:
            x: Target X position in KORUZA units
            y: Target Y position in KORUZA units
            f: Target F position in KORUZA units

        Returns:
            bool: True if move command accepted
        """
        # Stop any ongoing movement
        if self.move_thread and self.move_thread.is_alive():
            self.stop_flag = True
            self.move_thread.join(timeout=1.0)

        self.stop_flag = False

        # Set target positions
        with self.lock:
            self.target_position['x'] = x
            self.target_position['y'] = y
            self.target_position['f'] = f

        # Start movement thread
        self.move_thread = threading.Thread(target=self._move_thread_func)
        self.move_thread.daemon = True
        self.move_thread.start()

        logger.info("Move command: X=%d, Y=%d, F=%d", x, y, f)
        return True

    def configure(self, laser=None, speed=None, accel=None):
        """
        Configure motor parameters (interface compatible with I2C Motor class).

        Args:
            laser: Laser enable flag (0 or 1)
            speed: Motor speed in units/sec (not implemented in simple driver)
            accel: Motor acceleration in units/sec^2 (not implemented)

        Returns:
            bool: True if configuration successful
        """
        if laser is not None:
            self.laser_enabled = bool(laser)
            logger.info("Laser enabled: %s", self.laser_enabled)

        if speed is not None:
            self.speed = speed
            # Could adjust step_delay based on speed
            # step_delay = 1.0 / (speed * steps_per_unit)
            logger.info("Speed set: %d units/sec", speed)

        if accel is not None:
            self.accel = accel
            logger.info("Accel set: %d units/sec^2", accel)

        return True

    def home(self):
        """
        Home motors (set current position as origin).
        Note: No limit switches, so this is just a position reset.

        Returns:
            bool: True if homing successful
        """
        with self.lock:
            self.current_position = {'x': 0, 'y': 0, 'f': 0}
            self.target_position = {'x': 0, 'y': 0, 'f': 0}

        logger.info("Homing complete (position reset)")
        return True

    def cleanup(self):
        """Clean up GPIO resources."""
        # Stop any movement
        if self.move_thread and self.move_thread.is_alive():
            self.stop_flag = True
            self.move_thread.join(timeout=1.0)

        # Turn off all motors
        for axis in ['x', 'y', 'f']:
            self._motor_off(axis)

        # Don't call GPIO.cleanup() here as other processes might be using GPIO
        logger.info("GPIO Stepper Motor cleanup complete")

    def __del__(self):
        """Destructor."""
        try:
            self.cleanup()
        except:
            pass
