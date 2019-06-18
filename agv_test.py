import RPi.GPIO as GPIO
import time

# for front
CONTROL_PIN = 17
PWM_FREQ = 50
STEP = 15

GPIO.setmode(GPIO.BCM)
GPIO.setup(CONTROL_PIN, GPIO.OUT)

pwm = GPIO.PWM(CONTROL_PIN, PWM_FREQ)
pwm.start(0)

# for back
MOTOR_CONTROL_PIN1 = 22
MOTOR_CONTROL_PIN2 = 23
MOTOR_PWM_PIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(MOTOR_CONTROL_PIN1, GPIO.OUT)
GPIO.setup(MOTOR_CONTROL_PIN2, GPIO.OUT)
GPIO.setup(MOTOR_PWM_PIN, GPIO.OUT)

MOTER_PWM_FREQ = 1000
motor_pwm = GPIO.PWM(MOTOR_PWM_PIN, MOTER_PWM_FREQ)
motor_pwm.start(0)

GPIO.output(MOTOR_CONTROL_PIN1, False)
GPIO.output(MOTOR_CONTROL_PIN2, True)

# for guide path
def get_guide_path():
    path_dict = dict()
    red = dict()
    for i in [1, 2, 4, 7, 11, 13, 15]:
        red[i] = [3, 'r', 1, 'l', 1, 'r', 1]
    red[5] = [1, 'r', 1, 'o', 1, 'r', 2, 'r', 1, 'l', 1, 'r', 1]
    red[8] = [1, 'r', 1, 'l', 1, 'o', 1, 'r', 1, 'r', 2, 'r', 1, 'l', 1, 'r', 1]
    red[9] = [1, 'r', 2, 'o', 2, 'r', 2, 'r', 1, 'l', 1, 'r', 1]
    red[12] = [1, 'r', 2, 'l', 1, 'o', 1, 'r', 2, 'r', 2, 'r', 1, 'l', 1, 'r', 1]

    green = dict()
    for i in [1, 2, 3, 4, 5, 8, 11, 12, 13, 14, 15]:
        green[i] = [1, 'l', 1, 'l', 1, 'r', 1, 'r', 2, 'l', 1, 'l', 1, 'r', 1, 'r', 1]
    green[7] = [1, 'l', 1, 'l', 1, 'r', 2, 'o', 1, 'l', 2, 'l', 1, 'l', 1, 'r', 1, 'r', 1]

    blue = dict()
    for i in [1, 3, 5, 6, 8, 9, 12, 14, 15]:
        blue[i] = [2, 'l', 1, 'l', 1, 'r', 1, 'r', 1, 'l', 2]
    for i in [4, 7, 10, 13]:
        blue[i] = [2, 'l', 1, 'l', 1, 'r', 1, 'l', 1, 'r', 2, 'r', 2]
    blue[11] = [2, 'l', 1, 'l', 1, 'r', 2, 'l', 1, 'r', 1, 'r', 2]

    path_dict['red'] = red
    path_dict['green'] = green
    path_dict['blue'] = blue

    return path_dict


def angle_to_duty_cycle(angle=0):
    duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * angle / 180)
    return duty_cycle

def turn_angle(direction):
    if direction == "r":
        # turn right
        angle = 120
    elif direction == "l":
        # turn right
        angle = 60
    elif direction == "o":
        # turn opposite
        angle = 60
    else:
        return

    dc = angle_to_duty_cycle(angle)
    pwm.ChangeDutyCycle(dc)
    time.sleep(1)

    # turn back
    dc = angle_to_duty_cycle(90)
    pwm.ChangeDutyCycle(dc)

def guide(path):
    for step in path:
        if isinstance(step, int):
            # go direct
            motor_pwm.ChangeDutyCycle(50)
            t = step * 2 - 0.5
            time.sleep(t)
        else:
            # turn
            turn_angle(step)

def main():
    # color = "red"
    # station = 4
    # path_dict = get_guide_path()
    # path = path_dict[color][station]
    path = [1, 'l', 2, 'r', 1]
    guide(path)


if __name__ == "__main__":
    main()
