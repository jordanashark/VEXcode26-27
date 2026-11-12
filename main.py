from vex import *
import math

brain = Brain()
controller_1 = Controller(PRIMARY)

# =====================================================================
# HARDWARE
# =====================================================================
l_front = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
l_mid   = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
r_front = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
r_mid   = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
l_back  = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
r_back  = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)

left_drive  = MotorGroup(l_front, l_mid, l_back)
right_drive = MotorGroup(r_front, r_mid, r_back)
four_bar = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)

claw = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
imu = Inertial(Ports.PORT4)

# =====================================================================
# CONFIG  
# =====================================================================
# --- driver control ---
DEADBAND = 5
LOG_STRENGTH = 9
SINGLE_ARCADE = 0
SPLIT_ARCADE = 1
drive_mode = SINGLE_ARCADE
DRIVER_SLEW_PER_S = 400.0      # %/s the stick command may rise (decel is instant)

# --- drivetrain geometry  ---
WHEEL_DIAMETER = 3.25          # inches
DRIVE_GEAR_RATIO = 1.0         # wheel turns per motor turn (external gearing)
TRACK_WIDTH = 11.5             # inches between left and right wheel contact patches
IN_PER_DEG = DRIVE_GEAR_RATIO * math.pi * WHEEL_DIAMETER / 360.0
DRIVE_MAX_IPS = 200.0 * DRIVE_GEAR_RATIO * math.pi * WHEEL_DIAMETER / 60.0  # 200 rpm cartridge

# --- sensor fusion ---
IMU_WEIGHT = 0.98              # trust in IMU heading change vs wheel-derived heading change
SLIP_THRESHOLD_DEG = 4.0       # per update; bigger disagreement = wheels slipped, use IMU 100%

# --- drive motion (profile + PID + feedforward) ---
DRIVE_CRUISE_IPS = 0.65 * DRIVE_MAX_IPS
DRIVE_ACCEL_IPS2 = 45.0
DRIVE_KV = 100.0 / DRIVE_MAX_IPS   # % power per in/s of profile velocity
DRIVE_KP = 4.0
DRIVE_KI = 0.0
DRIVE_KD = 0.2
HEAD_KP = 2.0                  # heading-hold during straight drives
HEAD_KD = 0.1
HEAD_MAX = 30.0                # max steering correction (%)
DRIVE_SLEW_PER_S = 300.0       # %/s output slew in auton
DRIVE_TOL_IN = 0.5
DRIVE_SETTLE_VEL = 1.5         # in/s
DRIVE_SETTLE_MS = 120
DRIVE_EXTRA_TIMEOUT_MS = 1200

# --- turning ---
TURN_KP = 1.4
TURN_KI = 0.03
TURN_KD = 0.12
TURN_I_ZONE = 10.0
TURN_I_MAX = 12.0
TURN_MAX_POWER = 80.0
TURN_SLEW_PER_S = 600.0
TURN_TOL_DEG = 1.0
TURN_SETTLE_VEL = 5.0          # deg/s
TURN_SETTLE_MS = 120

# --- four bar ---
FOUR_BAR_MANUAL_POWER = 100
FOUR_BAR_SLEW_PER_S = 500.0
FOUR_BAR_KP = 0.6
FOUR_BAR_KI = 0.0
FOUR_BAR_KD = 0.02
FOUR_BAR_TOL_DEG = 3.0
FOUR_BAR_SETTLE_VEL = 20.0     # deg/s
FOUR_BAR_SETTLE_MS = 150

FOUR_BAR_LIMITS_ENABLED = False
FOUR_BAR_MIN_DEG = 0.0
FOUR_BAR_MAX_DEG = 600.0
FOUR_BAR_CARRY_DEG = 250.0     # example auton target

# --- claw ---
CLAW_CLOSE_POWER = 70
CLAW_OPEN_POWER = 50
CLAW_HOLD_POWER = 12           # trickle power to keep the grip without cooking the motor
CLAW_STALL_DPS = 15.0
CLAW_STALL_GRACE_MS = 150
CLAW_STALL_MS = 120

# --- logging ---
LOG_TO_SD = True               # needs an SD card in the brain; falls back to console only
LOG_FILE = "auton_log.csv"
LOG_FLUSH_ROWS = 60
LOG_PRINT_EVERY = 5            # print every Nth row to the terminal


# =====================================================================
# HELPERS
# =====================================================================
def now_ms():
    return brain.timer.time(MSEC)


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def wrap_180(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def apply_deadband(value):
    return 0 if abs(value) < DEADBAND else value


def log_scale(value):
    if value == 0:
        return 0
    sign = 1 if value > 0 else -1
    x = min(abs(value), 100) / 100.0
    scaled = math.log(1 + LOG_STRENGTH * x) / math.log(1 + LOG_STRENGTH)
    return sign * scaled * 100


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, i_zone=1e9, i_max=20.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_zone = i_zone
        self.i_max = i_max
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def update(self, err, dt):
        if dt < 0.001:
            dt = 0.001
        if self.ki != 0.0:
            if abs(err) < self.i_zone:
                self.integral += err * dt
                lim = self.i_max / self.ki
                self.integral = clamp(self.integral, -lim, lim)
            else:
                self.integral = 0.0
            if self.prev_err is not None and err * self.prev_err < 0:
                self.integral = 0.0   # crossed zero: dump windup
        deriv = 0.0
        if self.prev_err is not None:
            deriv = (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * deriv


class SlewLimiter:
    """Limits how fast a command may ramp up. Ramp-down is instant unless decel_rate is set."""
    def __init__(self, rate_per_s, decel_rate=None):
        self.rate = rate_per_s
        self.decel_rate = decel_rate
        self.value = 0.0

    def reset(self, value=0.0):
        self.value = value

    def update(self, target, dt):
        shrinking = (abs(target) <= abs(self.value)) and (target * self.value >= 0)
        if shrinking:
            if self.decel_rate is None:
                self.value = target
                return self.value
            step = self.decel_rate * dt
        else:
            step = self.rate * dt
        delta = clamp(target - self.value, -step, step)
        self.value += delta
        return self.value


class SettleDetector:
    """Done when error AND speed stay small for settle_ms in a row."""
    def __init__(self, pos_tol, vel_tol, settle_ms):
        self.pos_tol = pos_tol
        self.vel_tol = vel_tol
        self.settle_ms = settle_ms
        self.timer = 0.0

    def reset(self):
        self.timer = 0.0

    def update(self, err, vel, dt):
        if abs(err) <= self.pos_tol and abs(vel) <= self.vel_tol:
            self.timer += dt * 1000.0
        else:
            self.timer = 0.0
        return self.timer >= self.settle_ms


class TrapezoidProfile:
    """Distance profile with limited accel/decel. Falls back to a triangle on short moves."""
    def __init__(self, dist, vmax, accel):
        self.dist = dist
        self.a = accel
        if dist < (vmax * vmax) / accel:
            self.t_acc = math.sqrt(dist / accel) if dist > 0 else 0.0
            self.v_peak = accel * self.t_acc
            self.t_cruise = 0.0
        else:
            self.t_acc = vmax / accel
            self.v_peak = vmax
            d_acc = 0.5 * accel * self.t_acc * self.t_acc
            self.t_cruise = (dist - 2 * d_acc) / vmax
        self.d_acc = 0.5 * accel * self.t_acc * self.t_acc
        self.total = 2 * self.t_acc + self.t_cruise

    def get(self, t):
        if t <= 0:
            return 0.0, 0.0
        if t >= self.total:
            return self.dist, 0.0
        if t < self.t_acc:
            return 0.5 * self.a * t * t, self.a * t
        if t < self.t_acc + self.t_cruise:
            tc = t - self.t_acc
            return self.d_acc + self.v_peak * tc, self.v_peak
        td = self.total - t
        return self.dist - 0.5 * self.a * td * td, self.a * td


class Logger:
    def __init__(self, filename):
        self.filename = filename
        self.rows = []
        self.count = 0
        self.sd_ok = False

    def start(self):
        self.rows = []
        self.count = 0
        self.sd_ok = False
        if LOG_TO_SD:
            try:
                self.sd_ok = brain.sdcard.is_inserted()
            except:
                self.sd_ok = False
        header = "ms,tag,x,y,theta,imu,target,measured,error,output\n"
        if self.sd_ok:
            try:
                brain.sdcard.savefile(self.filename, bytearray(header, 'utf-8'))
            except:
                self.sd_ok = False
        print(header.strip())

    def row(self, tag, target, measured, error, output):
        line = "%d,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f\n" % (
            now_ms(), tag, odom.x, odom.y, odom.theta, odom.imu_rotation,
            target, measured, error, output)
        self.count += 1
        if self.count % LOG_PRINT_EVERY == 0:
            print(line.strip())
        if self.sd_ok:
            self.rows.append(line)
            if len(self.rows) >= LOG_FLUSH_ROWS:
                self.flush()

    def flush(self):
        if self.sd_ok and len(self.rows) > 0:
            try:
                brain.sdcard.appendfile(self.filename, bytearray("".join(self.rows), 'utf-8'))
            except:
                self.sd_ok = False
            self.rows = []


# =====================================================================
# ODOMETRY + SENSOR FUSION
# =====================================================================
class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0           # degrees, continuous (not wrapped)
        self.v = 0.0               # in/s, filtered
        self.omega = 0.0           # deg/s, filtered
        self.imu_rotation = 0.0
        self.imu_ok = False
        self.slipping = False
        self.last_l = 0.0
        self.last_r = 0.0
        self.last_imu = 0.0
        self.last_t = 0

    def _wheels(self):
        l = (l_front.position(DEGREES) + l_mid.position(DEGREES) + l_back.position(DEGREES)) / 3.0
        r = (r_front.position(DEGREES) + r_mid.position(DEGREES) + r_back.position(DEGREES)) / 3.0
        return l * IN_PER_DEG, r * IN_PER_DEG

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.imu_ok = imu.installed() and not imu.is_calibrating()
        if self.imu_ok:
            imu.set_rotation(theta, DEGREES)
            self.imu_rotation = theta
            self.last_imu = theta
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.last_l, self.last_r = self._wheels()
        self.last_t = now_ms()

    def update(self):
        t = now_ms()
        dt = (t - self.last_t) / 1000.0
        if dt <= 0:
            return
        self.last_t = t

        l, r = self._wheels()
        dl = l - self.last_l
        dr = r - self.last_r
        self.last_l = l
        self.last_r = r

        d = (dl + dr) / 2.0
        enc_dth = math.degrees((dl - dr) / TRACK_WIDTH)   # right farther than left = turning left = negative

        if self.imu_ok:
            rot = imu.rotation(DEGREES)
            imu_dth = rot - self.last_imu
            self.last_imu = rot
            self.imu_rotation = rot
            w = IMU_WEIGHT
            self.slipping = abs(imu_dth - enc_dth) > SLIP_THRESHOLD_DEG
            if self.slipping:
                w = 1.0
            dth = w * imu_dth + (1.0 - w) * enc_dth
        else:
            dth = enc_dth

        mid = math.radians(self.theta + dth / 2.0)
        self.x += d * math.sin(mid)
        self.y += d * math.cos(mid)
        self.theta += dth

        self.v = 0.7 * self.v + 0.3 * (d / dt)
        self.omega = 0.7 * self.omega + 0.3 * (dth / dt)


odom = Odometry()
log = Logger(LOG_FILE)
odom_thread = None


def odometry_loop():
    tick = 0
    while True:
        odom.update()
        tick += 1
        if tick % 25 == 0:   # ~4 Hz status
            brain.screen.set_cursor(3, 1)
            brain.screen.print("%-34s" % ("X %.1f  Y %.1f  H %.1f" % (odom.x, odom.y, odom.theta)))
            brain.screen.set_cursor(4, 1)
            brain.screen.print("%-34s" % ("4bar %.0f  claw %.0f" % (four_bar.position(DEGREES), claw.position(DEGREES))))
        wait(10, MSEC)


# =====================================================================
# DRIVE PRIMITIVES
# =====================================================================
def set_drive_stopping(mode):
    left_drive.set_stopping(mode)
    right_drive.set_stopping(mode)


def drive_arcade(forward, turn):
    left_power = forward + turn
    right_power = forward - turn
    max_mag = max(abs(left_power), abs(right_power))
    if max_mag > 100:
        left_power = left_power * 100 / max_mag
        right_power = right_power * 100 / max_mag
    if left_power == 0 and right_power == 0:
        left_drive.stop()
        right_drive.stop()
    else:
        left_drive.spin(FORWARD, left_power, PERCENT)
        right_drive.spin(FORWARD, right_power, PERCENT)


dist_pid = PID(DRIVE_KP, DRIVE_KI, DRIVE_KD, i_zone=3.0, i_max=15.0)
head_pid = PID(HEAD_KP, 0.0, HEAD_KD)
turn_pid = PID(TURN_KP, TURN_KI, TURN_KD, i_zone=TURN_I_ZONE, i_max=TURN_I_MAX)
fwd_slew = SlewLimiter(DRIVE_SLEW_PER_S)
turn_slew = SlewLimiter(TURN_SLEW_PER_S)


def drive_distance(inches, max_ips=None, accel=None, timeout_ms=None, tol=DRIVE_TOL_IN):
    """Profiled straight drive (negative = reverse). Holds the starting heading."""
    if max_ips is None:
        max_ips = DRIVE_CRUISE_IPS
    if accel is None:
        accel = DRIVE_ACCEL_IPS2
    sign = 1 if inches >= 0 else -1
    profile = TrapezoidProfile(abs(inches), max_ips, accel)
    if timeout_ms is None:
        timeout_ms = int(profile.total * 1000) + DRIVE_EXTRA_TIMEOUT_MS

    h0 = odom.theta
    x0 = odom.x
    y0 = odom.y
    sin_h = math.sin(math.radians(h0))
    cos_h = math.cos(math.radians(h0))

    dist_pid.reset()
    head_pid.reset()
    fwd_slew.reset(0.0)
    settle = SettleDetector(tol, DRIVE_SETTLE_VEL, DRIVE_SETTLE_MS)
    set_drive_stopping(BRAKE)

    start = now_ms()
    last = start
    reason = "timeout"
    while True:
        wait(10, MSEC)
        t = now_ms()
        dt = (t - last) / 1000.0
        last = t
        elapsed = (t - start) / 1000.0

        sp, sv = profile.get(elapsed)
        sp *= sign
        sv *= sign
        traveled = (odom.x - x0) * sin_h + (odom.y - y0) * cos_h
        err = sp - traveled
        final_err = inches - traveled

        out = DRIVE_KV * sv + dist_pid.update(err, dt)
        out = clamp(out, -100, 100)
        out = fwd_slew.update(out, dt)
        steer = clamp(head_pid.update(wrap_180(h0 - odom.theta), dt), -HEAD_MAX, HEAD_MAX)
        drive_arcade(out, steer)
        log.row("drive", inches, traveled, final_err, out)

        if elapsed >= profile.total and settle.update(final_err, odom.v, dt):
            reason = "settled"
            break
        if (t - start) >= timeout_ms:
            break

    left_drive.stop()
    right_drive.stop()
    log.row("drive_" + reason, inches, 0, 0, 0)
    log.flush()
    return reason == "settled"


def _turn(target, wrap, timeout_ms, tol, max_power):
    if max_power is None:
        max_power = TURN_MAX_POWER
    turn_pid.reset()
    turn_slew.reset(0.0)
    settle = SettleDetector(tol, TURN_SETTLE_VEL, TURN_SETTLE_MS)
    set_drive_stopping(BRAKE)

    start = now_ms()
    last = start
    reason = "timeout"
    while True:
        wait(10, MSEC)
        t = now_ms()
        dt = (t - last) / 1000.0
        last = t

        err = target - odom.theta
        if wrap:
            err = wrap_180(err)
        out = clamp(turn_pid.update(err, dt), -max_power, max_power)
        out = turn_slew.update(out, dt)
        drive_arcade(0, out)
        log.row("turn", target, odom.theta, err, out)

        if settle.update(err, odom.omega, dt):
            reason = "settled"
            break
        if (t - start) >= timeout_ms:
            break

    left_drive.stop()
    right_drive.stop()
    log.row("turn_" + reason, target, odom.theta, 0, 0)
    log.flush()
    return reason == "settled"


def turn_to(heading_deg, timeout_ms=1800, tol=TURN_TOL_DEG, max_power=None):
    """Turn to an absolute field heading by the shortest way."""
    return _turn(heading_deg, True, timeout_ms, tol, max_power)


def turn_by(delta_deg, timeout_ms=1800, tol=TURN_TOL_DEG, max_power=None):
    """Turn by a relative amount; can exceed 180 degrees."""
    return _turn(odom.theta + delta_deg, False, timeout_ms, tol, max_power)


def move_to_point(x, y, **kwargs):
    """Face the point, then drive to it."""
    dx = x - odom.x
    dy = y - odom.y
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.5:
        return True
    heading = math.degrees(math.atan2(dx, dy))
    turn_to(heading)
    return drive_distance(dist, **kwargs)


# =====================================================================
# FOUR BAR
# =====================================================================
fb_pid = PID(FOUR_BAR_KP, FOUR_BAR_KI, FOUR_BAR_KD, i_zone=20.0, i_max=15.0)
fb_slew = SlewLimiter(FOUR_BAR_SLEW_PER_S)


def four_bar_goto(target_deg, timeout_ms=1500, max_power=100):
    """PID the four bar to a position (degrees from its power-on position), then HOLD."""
    if FOUR_BAR_LIMITS_ENABLED:
        target_deg = clamp(target_deg, FOUR_BAR_MIN_DEG, FOUR_BAR_MAX_DEG)
    fb_pid.reset()
    fb_slew.reset(0.0)
    settle = SettleDetector(FOUR_BAR_TOL_DEG, FOUR_BAR_SETTLE_VEL, FOUR_BAR_SETTLE_MS)

    start = now_ms()
    last = start
    reason = "timeout"
    while True:
        wait(10, MSEC)
        t = now_ms()
        dt = (t - last) / 1000.0
        last = t

        pos = four_bar.position(DEGREES)
        err = target_deg - pos
        out = clamp(fb_pid.update(err, dt), -max_power, max_power)
        out = fb_slew.update(out, dt)
        four_bar.spin(FORWARD, out, PERCENT)
        log.row("fourbar", target_deg, pos, err, out)

        if settle.update(err, four_bar.velocity(DPS), dt):
            reason = "settled"
            break
        if (t - start) >= timeout_ms:
            break

    four_bar.stop(HOLD)
    log.row("fourbar_" + reason, target_deg, four_bar.position(DEGREES), 0, 0)
    log.flush()
    return reason == "settled"


# =====================================================================
# CLAW 
# =====================================================================
claw_gripping = False


def _claw_run(forward, power, timeout_ms):
    claw.spin(FORWARD if forward else REVERSE, power, PERCENT)
    start = now_ms()
    last = start
    stalled_ms = 0
    while (now_ms() - start) < timeout_ms:
        wait(10, MSEC)
        t = now_ms()
        dt_ms = t - last
        last = t
        if (t - start) > CLAW_STALL_GRACE_MS and abs(claw.velocity(DPS)) < CLAW_STALL_DPS:
            stalled_ms += dt_ms
        else:
            stalled_ms = 0
        if stalled_ms >= CLAW_STALL_MS:
            return True
    return False


def claw_close(timeout_ms=900):
    global claw_gripping
    stalled = _claw_run(True, CLAW_CLOSE_POWER, timeout_ms)
    claw.spin(FORWARD, CLAW_HOLD_POWER, PERCENT)
    claw_gripping = True
    log.row("claw_close_stall" if stalled else "claw_close_timeout", 0, claw.position(DEGREES), 0, CLAW_HOLD_POWER)
    return stalled


def claw_open(timeout_ms=700):
    global claw_gripping
    stalled = _claw_run(False, CLAW_OPEN_POWER, timeout_ms)
    claw.stop(HOLD)
    claw_gripping = False
    log.row("claw_open_stall" if stalled else "claw_open_timeout", 0, claw.position(DEGREES), 0, 0)
    return stalled


# =====================================================================
# DRIVER CONTROL HELPERS
# =====================================================================
def show_mode():
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    if drive_mode == SINGLE_ARCADE:
        controller_1.screen.print("Single Arcade (L)")
    else:
        controller_1.screen.print("Split Arcade")


# =====================================================================
# COMPETITION FUNCTIONS
# =====================================================================
def pre_auton():
    global odom_thread
    set_drive_stopping(BRAKE)
    left_drive.set_velocity(0, PERCENT)
    right_drive.set_velocity(0, PERCENT)
    four_bar.set_stopping(HOLD)
    claw.set_stopping(HOLD)
    show_mode()

    # IMU must sit still while it calibrates (~2 s)
    if imu.installed():
        imu.calibrate()
        waited = 0
        while imu.is_calibrating() and waited < 5000:
            wait(50, MSEC)
            waited += 50

    # Start with the four bar at the bottom and the claw in its start position
    four_bar.set_position(0, DEGREES)
    claw.set_position(0, DEGREES)
    for m in (l_front, l_mid, l_back, r_front, r_mid, r_back):
        m.set_position(0, DEGREES)

    odom.reset(0.0, 0.0, 0.0)
    if odom_thread is None:
        odom_thread = Thread(odometry_loop)


def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Running")
    log.start()

    # Put HERE ---

    # ---------------------------------------------------------------

    log.flush()


def usercontrol():
    global drive_mode, claw_gripping
    set_drive_stopping(BRAKE)
    last_a = False
    show_mode()

    fwd_slew_d = SlewLimiter(DRIVER_SLEW_PER_S)
    turn_slew_d = SlewLimiter(DRIVER_SLEW_PER_S)
    fb_slew_d = SlewLimiter(FOUR_BAR_SLEW_PER_S)
    fb_state = 0          # -1 down, 0 holding, 1 up
    claw_state = "idle"
    last_t = now_ms()

    while True:
        t = now_ms()
        dt = (t - last_t) / 1000.0
        last_t = t

        # ----- drive -----
        current_a = controller_1.buttonA.pressing()
        if current_a and not last_a:
            drive_mode = SPLIT_ARCADE if drive_mode == SINGLE_ARCADE else SINGLE_ARCADE
            show_mode()
        last_a = current_a

        if drive_mode == SINGLE_ARCADE:
            fwd_raw = controller_1.axis3.position()
            turn_raw = controller_1.axis4.position()
        else:
            fwd_raw = controller_1.axis3.position()
            turn_raw = controller_1.axis1.position()

        forward = log_scale(apply_deadband(fwd_raw))
        turn = log_scale(apply_deadband(turn_raw))
        forward = fwd_slew_d.update(forward, dt)
        turn = turn_slew_d.update(turn, dt)
        drive_arcade(forward, turn)

        # ----- four bar: R1 up, R2 down, HOLD when released -----
        up = controller_1.buttonR1.pressing()
        down = controller_1.buttonR2.pressing()
        pos = four_bar.position(DEGREES)
        if FOUR_BAR_LIMITS_ENABLED:
            if pos >= FOUR_BAR_MAX_DEG:
                up = False
            if pos <= FOUR_BAR_MIN_DEG:
                down = False
        if up and not down:
            target = FOUR_BAR_MANUAL_POWER
        elif down and not up:
            target = -FOUR_BAR_MANUAL_POWER
        else:
            target = 0
        fb_cmd = fb_slew_d.update(target, dt)
        if target == 0:
            if fb_state != 0:
                four_bar.stop(HOLD)
                fb_state = 0
        else:
            four_bar.spin(FORWARD, fb_cmd, PERCENT)
            fb_state = 1 if target > 0 else -1

        # ----- claw: L1 close, L2 open, trickle-hold after closing -----
        close_btn = controller_1.buttonL1.pressing()
        open_btn = controller_1.buttonL2.pressing()
        if close_btn and not open_btn:
            claw.spin(FORWARD, CLAW_CLOSE_POWER, PERCENT)
            claw_gripping = True
            claw_state = "closing"
        elif open_btn and not close_btn:
            claw.spin(REVERSE, CLAW_OPEN_POWER, PERCENT)
            claw_gripping = False
            claw_state = "opening"
        elif claw_gripping:
            claw.spin(FORWARD, CLAW_HOLD_POWER, PERCENT)
            claw_state = "holding"
        elif claw_state != "idle":
            claw.stop(HOLD)
            claw_state = "idle"

        wait(20, MSEC)


competition = Competition(usercontrol, autonomous)
pre_auton()
