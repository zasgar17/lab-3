#!/usr/bin/env pybricks-micropython
"""EV3 MicroPython v2: real P / PD / PID distance control and CSV logging.
B = left motor; C = right motor; S4 = ultrasonic sensor.
LEFT/RIGHT select a menu item; CENTER confirms; DOWN finishes a run and saves its CSV. No run time limit.
The gains below are STARTING VALUES, not experimentally tuned results.
Motor commands and logged u are deg/s, NOT mm/s.
"""
from pybricks.ev3devices import UltrasonicSensor, Motor
from pybricks.parameters import Port, Button
from pybricks.hubs import EV3Brick
from pybricks.tools import wait, StopWatch, DataLog

SETPOINT_MM = 200
PERIOD_MS = 50
MAX_SPEED_DPS = 300
MIN_VALID_MM = 30
MAX_VALID_MM = 2500
DERIVATIVE_FILTER_SECONDS = 0.15
# controller: (Kp, Ki, Kd). Keep a written record of any changes.
# Units: Kp = (deg/s)/mm; Ki = (deg/s)/(mm*s); Kd = deg/mm.
GAINS = {'P': (1.0, 0.0, 0.0),
         'PD': (1.0, 0.0, 0.15),
         'PID': (1.0, 0.15, 0.15)}


def clamp(value, limit):
    return max(-limit, min(limit, value))


class DistanceController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.derivative = 0.0

    def update(self, error, dt):
        dt = max(dt, 0.001)
        raw_d = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        alpha = dt / (DERIVATIVE_FILTER_SECONDS + dt)
        self.derivative += alpha * (raw_d - self.derivative)
        # Conditional integration: reject only an integral change that pushes
        # a saturated command farther into saturation; permit unwinding.
        if self.ki != 0:
            candidate_i = self.integral + error * dt
            candidate_u = self.kp * error + self.ki * candidate_i + self.kd * self.derivative
            pushes_high = candidate_u > MAX_SPEED_DPS and error > 0
            pushes_low = candidate_u < -MAX_SPEED_DPS and error < 0
            if not (pushes_high or pushes_low):
                self.integral = candidate_i
        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * self.derivative
        raw_u = p + i + d
        self.previous_error = error
        return clamp(raw_u, MAX_SPEED_DPS), raw_u, p, i, d


def show(ev3, lines):
    ev3.screen.clear()
    for line in lines:
        ev3.screen.print(line)


def release_buttons(ev3):
    while ev3.buttons.pressed():
        wait(20)


def choose(ev3, title, options):
    index = 0
    release_buttons(ev3)
    while True:
        show(ev3, [title, options[index], 'LEFT / RIGHT', 'CENTER = select'])
        buttons = ev3.buttons.pressed()
        if Button.LEFT in buttons:
            index = (index - 1) % len(options)
            release_buttons(ev3)
        elif Button.RIGHT in buttons:
            index = (index + 1) % len(options)
            release_buttons(ev3)
        elif Button.CENTER in buttons:
            release_buttons(ev3)
            return index
        wait(30)


def brake_both(left, right):
    try:
        left.brake()
    finally:
        right.brake()


def run_test(ev3, sensor, left, right, mode, test):
    gains = GAINS[mode]
    controller = DistanceController(*gains)
    expected_start = 600 if test == 'A_step' else 200
    show(ev3, [mode + ' / ' + test, 'Sensor gap: %d mm' % expected_start,
               'A: fixed target', 'B/C: move manually', 'DOWN = finish', 'CENTER = ready'])
    while Button.CENTER not in ev3.buttons.pressed():
        wait(20)
    release_buttons(ev3)
    # A separate timestamped file is created for every trial, including repeats.
    log = DataLog('t_s', 'dt_s', 'controller', 'test', 'phase', 'event',
                  'distance_mm', 'error_mm', 'u_dps', 'u_raw_dps',
                  'p_dps', 'i_dps', 'd_dps', 'kp', 'ki', 'kd',
                  'valid_echo', 'saturated', 'left_speed_dps', 'right_speed_dps',
                  'left_angle_deg', 'right_angle_deg', 'status',
                  name='task3_' + mode + '_' + test, timestamp=True)
    print('Created CSV: task3_' + mode + '_' + test + '_<timestamp>.csv')
    print('Gains:', gains, 'Motor command limit:', MAX_SPEED_DPS, 'deg/s')
    print('No automatic time limit. DOWN finishes. RIGHT marks target moving; LEFT marks target stopped.')
    for number in [3, 2, 1]:
        show(ev3, ['Starting in', str(number), 'Keep target still'])
        ev3.speaker.beep(500, 80)
        wait(920)
    ev3.speaker.beep(900, 100)
    clock = StopWatch()
    previous_ms = None
    display_ms = -1000
    phase = 'stationary_target' if test == 'A_step' else 'unmarked'
    previous_buttons = []
    outcome = 'finished_by_user'
    try:
        while True:
            loop_ms = clock.time()
            buttons = ev3.buttons.pressed()
            abort = Button.DOWN in buttons
            if abort:
                brake_both(left, right)
            events = []
            # Optional operator markers. They never change the control law.
            # Have a partner mark target movement; use video for exact timing.
            if Button.RIGHT in buttons and Button.RIGHT not in previous_buttons:
                events.append('operator_move')
                if test != 'A_step':
                    phase = 'move_target_away'
            if Button.LEFT in buttons and Button.LEFT not in previous_buttons:
                events.append('operator_stop')
                if test != 'A_step':
                    phase = 'hold_after_stop'
            previous_buttons = buttons
            distance = sensor.distance()
            sample_ms = clock.time()
            dt = PERIOD_MS / 1000 if previous_ms is None else (sample_ms - previous_ms) / 1000
            previous_ms = sample_ms
            valid = MIN_VALID_MM <= distance < MAX_VALID_MM
            error = distance - SETPOINT_MM
            if abort:
                outcome = 'finished_by_user'
                events.append('user_finish')
            if valid and not abort:
                u, raw_u, p, i, d = controller.update(error, dt)
                left.run(u)
                right.run(u)
                status = 'running'
            else:
                brake_both(left, right)
                controller.reset()
                u = raw_u = p = i = d = 0.0
                status = 'finished_by_user' if abort else 'invalid_echo_braked'
            log.log(sample_ms / 1000, dt, mode, test, phase, '|'.join(events),
                    distance, error, u, raw_u, p, i, d, gains[0], gains[1], gains[2],
                    int(valid), int(valid and abs(raw_u) > MAX_SPEED_DPS),
                    left.speed(), right.speed(), left.angle(), right.angle(), status)
            if sample_ms - display_ms >= 250:
                show(ev3, [mode + ' / ' + test, 't: %.1f s' % (sample_ms / 1000),
                           'd: %d mm' % distance, 'u: %.1f deg/s' % u,
                           phase, 'DOWN = finish'])
                display_ms = sample_ms
            if abort:
                break
            wait(max(1, PERIOD_MS - (clock.time() - loop_ms)))
    except BaseException:
        outcome = 'error_or_interrupt'
        raise
    finally:
        brake_both(left, right)
        log.log(clock.time() / 1000, 0, mode, test, 'end', 'run_end',
                '', '', 0, '', '', '', '', gains[0], gains[1], gains[2],
                '', '', left.speed(), right.speed(), left.angle(), right.angle(), outcome)
    ev3.speaker.beep(800, 150)
    show(ev3, ['Run: ' + outcome, 'CSV saved on EV3', 'Download in VS Code', 'CENTER = menu'])
    release_buttons(ev3)
    while Button.CENTER not in ev3.buttons.pressed():
        wait(20)
    release_buttons(ev3)


def main():
    ev3 = EV3Brick()
    sensor = UltrasonicSensor(Port.S4)
    left, right = Motor(Port.B), Motor(Port.C)
    try:
        while True:
            mode = ['P', 'PD', 'PID'][choose(ev3, 'Controller', ['P', 'PD', 'PID'])]
            test = ['A_step', 'B_follow', 'C_stop'][choose(
                ev3, 'Test', ['A: step 600->200', 'B: follow moving', 'C: sudden stop'])]
            run_test(ev3, sensor, left, right, mode, test)
    finally:
        brake_both(left, right)


if __name__ == '__main__':
    main()
