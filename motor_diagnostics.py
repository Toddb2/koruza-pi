#!/usr/bin/env python
"""
Motor & IPC Diagnostics for KORUZA GPIO stepper setup.

Run on the Raspberry Pi as root to check the full stack:
    sudo python motor_diagnostics.py

Tests (in order):
  1. RPi.GPIO import
  2. GPIOStepperMotor init (pin setup)
  3. motor.read() returns valid dict
  4. Single-step movement on each axis
  5. IPC connectivity to koruza-sensors (if running)

Exit code 0 = all tests passed.
"""

import sys
import time
import traceback

PASS = '[PASS]'
FAIL = '[FAIL]'
SKIP = '[SKIP]'

# GPIO pin assignments (must match koruza-sensors initialize_motor)
PINS_X = (5,  6,  13, 26)
PINS_Y = (12, 16, 20, 21)
PINS_F = (17, 27, 22, 23)

REQUIRED_KEYS = (
    'current_x', 'current_y', 'current_f',
    'next_x',    'next_y',    'next_f',
    'status_x',  'status_y',  'status_f',
    'flash_status', 'flash_write_count',
    'command', 'laser', 'speed', 'accel', 'empty',
)

failures = []


def check(label, ok, detail=''):
    tag = PASS if ok else FAIL
    print '%s %s%s' % (tag, label, (' — ' + detail) if detail else '')
    if not ok:
        failures.append(label)
    return ok


# ---------------------------------------------------------------------------
# 0. Pre-flight: warn if koruza-sensors is already running (GPIO conflict)
# ---------------------------------------------------------------------------
import os, subprocess
print '\n=== 0. Pre-flight checks ==='
try:
    out = subprocess.check_output(['sv', 'status', 'koruza-sensors'],
                                  stderr=subprocess.STDOUT)
    if 'run:' in out:
        print 'WARNING: koruza-sensors is currently running.'
        print '         It holds the GPIO pins — this script will fail at test 2.'
        print '         Stop it first:  sudo service koruza-sensors stop'
        print '         (IPC test in section 5 will be skipped as a result.)'
    else:
        print '%s koruza-sensors is not running' % PASS
except Exception:
    # sv not installed or service not found — not a problem, just skip
    print '%s sv check skipped (runit not available here)' % SKIP

# ---------------------------------------------------------------------------
# 1. RPi.GPIO import
# ---------------------------------------------------------------------------
print '\n=== 1. GPIO import ==='
try:
    import RPi.GPIO as GPIO
    check('RPi.GPIO importable', True)
except ImportError, e:
    check('RPi.GPIO importable', False, str(e))
    print 'Cannot continue without RPi.GPIO.'
    sys.exit(1)

# ---------------------------------------------------------------------------
# 2. GPIOStepperMotor init
# ---------------------------------------------------------------------------
print '\n=== 2. Motor driver init ==='
sys.path.insert(0, '/koruza')
motor = None
try:
    from drivers.gpio_stepper import GPIOStepperMotor
    check('gpio_stepper import', True)
    motor = GPIOStepperMotor(
        pins_x=PINS_X, pins_y=PINS_Y, pins_f=PINS_F,
        max_x=35000, max_y=35000, max_f=20000,
        steps_per_unit=1.0, step_delay=0.002,
    )
    check('GPIOStepperMotor() constructor', True)
except Exception, e:
    check('GPIOStepperMotor() constructor', False, str(e))
    traceback.print_exc()
    motor = None

# ---------------------------------------------------------------------------
# 3. read() validity
# ---------------------------------------------------------------------------
print '\n=== 3. motor.read() ==='
if motor is None:
    print '%s motor.read() — motor not initialised, skipping' % SKIP
else:
    try:
        status = motor.read()
        check('read() returns a dict', isinstance(status, dict))
        check('read() not None', status is not None)

        missing = [k for k in REQUIRED_KEYS if k not in status]
        check('all required keys present', not missing,
              ('missing: ' + ', '.join(missing)) if missing else '')

        check("'empty' == 255", status.get('empty') == 255,
              'got %r' % status.get('empty'))

        print '    current  X=%d Y=%d F=%d' % (
            status['current_x'], status['current_y'], status['current_f'])
        print '    status   X=%d Y=%d F=%d' % (
            status['status_x'], status['status_y'], status['status_f'])
    except Exception, e:
        check('read() ran without exception', False, str(e))
        traceback.print_exc()

# ---------------------------------------------------------------------------
# 4. Short movement test (10 steps each axis, then back)
# ---------------------------------------------------------------------------
print '\n=== 4. Movement test (10 units each axis) ==='
if motor is None:
    print '%s movement — motor not initialised, skipping' % SKIP
else:
    def wait_idle(timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            s = motor.read()
            if s['status_x'] == 0 and s['status_y'] == 0 and s['status_f'] == 0:
                return True
            time.sleep(0.05)
        return False

    s = motor.read()
    cx = s['current_x']
    cy = s['current_y']
    cf = s['current_f']

    for label, dx, dy, df in [
        ('X-axis 10-unit move', 10, 0, 0),
        ('Y-axis 10-unit move', 0, 10, 0),
        ('F-axis 10-unit move', 0, 0, 10),
    ]:
        try:
            motor.move(x=cx + dx, y=cy + dy, f=cf + df)
            idle = wait_idle()
            check(label + ' (outward)', idle, 'timed out still moving' if not idle else '')
            motor.move(x=cx, y=cy, f=cf)
            wait_idle()
            check(label + ' (return)', True)
        except Exception, e:
            check(label, False, str(e))
            traceback.print_exc()

    motor.cleanup()
    print '    GPIO released.'

# ---------------------------------------------------------------------------
# 5. IPC connectivity (koruza-sensors must be running)
# ---------------------------------------------------------------------------
print '\n=== 5. IPC connectivity (requires koruza-sensors) ==='
try:
    import nnpy
    s = nnpy.Socket(nnpy.AF_SP, nnpy.REQ)
    s.setsockopt(nnpy.SOL_SOCKET, nnpy.RCVTIMEO, 2000)  # 2 s timeout
    s.connect('ipc:///tmp/koruza-command.ipc')
    import json
    s.send(json.dumps({'type': 'command', 'command': 'get_status'}))
    try:
        reply = json.loads(s.recv())
        ok = reply.get('type') == 'cmd_reply'
        check('get_status reply received', ok, str(reply.get('type')))
        if ok:
            motor_meta = reply.get('motor')
            check('motor key in status reply', motor_meta is not None,
                  'None means motor init failed in koruza-sensors')
    except Exception, e:
        check('IPC recv', False, str(e))
    s.close()
except ImportError:
    print '%s IPC test — nnpy not installed' % SKIP
except Exception, e:
    check('IPC connect', False, str(e))
    print '    Is koruza-sensors running?  sudo service koruza-sensors start'

# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------
print '\n=== Result ==='
if failures:
    print 'FAILED — %d check(s) failed: %s' % (len(failures), ', '.join(failures))
    sys.exit(1)
else:
    print 'All checks passed.'
    sys.exit(0)
