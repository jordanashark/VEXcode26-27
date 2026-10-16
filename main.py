from vex import *
import math

brain = Brain()
controller_1 = Controller(PRIMARY)

l_front = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
l_mid   = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r_front = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
r_mid   = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)
l_back  = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
r_back  = Motor(Ports.PORT6, GearSetting.RATIO_18_1, True)

left_drive  = MotorGroup(l_front, l_mid, l_back)
right_drive = MotorGroup(r_front, r_mid, r_back)

DEADBAND = 5
LOG_STRENGTH = 9

SINGLE_ARCADE = 0
SPLIT_ARCADE = 1
drive_mode = SINGLE_ARCADE


def apply_deadband(value):
    return 0 if abs(value) < DEADBAND else value


def log_scale(value):
    if value == 0:
        return 0
    sign = 1 if value > 0 else -1
    x = min(abs(value), 100) / 100.0
    scaled = math.log(1 + LOG_STRENGTH * x) / math.log(1 + LOG_STRENGTH)
    return sign * scaled * 100


def show_mode():
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    if drive_mode == SINGLE_ARCADE:
        controller_1.screen.print("Single Arcade (L)")
    else:
        controller_1.screen.print("Split Arcade")


def set_drive_stopping(mode):
    left_drive.set_stopping(mode)
    right_drive.set_stopping(mode)


def pre_auton():
    set_drive_stopping(BRAKE)
    left_drive.set_velocity(0, PERCENT)
    right_drive.set_velocity(0, PERCENT)
    show_mode()


def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("Autonomous Running")


def usercontrol():
    global drive_mode
    set_drive_stopping(BRAKE)
    last_a = False
    show_mode()

    while True:
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

        wait(20, MSEC)


competition = Competition(usercontrol, autonomous)
pre_auton()
