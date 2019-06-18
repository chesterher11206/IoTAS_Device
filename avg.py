import time
import RPi.GPIO as GPIO


class AVG(object):
    def __init__(self, *args, **kwargs):
        self.front()
        self.back()
        self.ultrasonic()
        self.get_guide_path()

    def front(self):
        self.CONTROL_PIN = 17
        self.PWM_FREQ = 50

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.CONTROL_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(self.CONTROL_PIN, self.PWM_FREQ)
        self.pwm.start(0)

    def back(self):
        self.MOTOR_CONTROL_PIN1 = 22
        self.MOTOR_CONTROL_PIN2 = 23
        self.MOTOR_PWM_PIN = 24

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.MOTOR_CONTROL_PIN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_CONTROL_PIN2, GPIO.OUT)
        GPIO.setup(self.MOTOR_PWM_PIN, GPIO.OUT)

        self.MOTER_PWM_FREQ = 1000
        self.motor_pwm = GPIO.PWM(self.MOTOR_PWM_PIN, self.MOTER_PWM_FREQ)
        self.motor_pwm.start(0)

        GPIO.output(self.MOTOR_CONTROL_PIN1, False)
        GPIO.output(self.MOTOR_CONTROL_PIN2, True)

    def ultrasonic():
        self.TRIGGER_PIN = 23
        self.ECHO_PIN = 24

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)

    def get_guide_path(self):
        self.path_dict = dict()
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

        self.path_dict['red'] = red
        self.path_dict['green'] = green
        self.path_dict['blue'] = blue

    def angle_to_duty_cycle(self, angle=0):
        duty_cycle = (0.05 * self.PWM_FREQ) + (0.19 * self.PWM_FREQ * angle / 180)
        return duty_cycle

    def turn_angle(self, direction):
        if direction == "r":
            # turn right
            angle = 0
        elif direction == "l":
            # turn right
            angle = 0
        elif direction == "o":
            # turn opposite
            angle = 0
        else:
            return

        dc = self.angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(dc)

        # turn back
        time.sleep(2)
        dc = self.angle_to_duty_cycle(0)
        self.pwm.ChangeDutyCycle(dc)

    def start_guide(self, path):
        for step in path:
            if isinstance(step, int):
                # go direct
                self.motor_pwm.ChangeDutyCycle(50)
                time.sleep(5)
            else:
                # turn
                self.turn_angle(step)

    def send_trigger_pulse(self):
        GPIO.output(self.TRIGGER_PIN, True)
        time.sleep(0.001)
        GPIO.output(self.TRIGGER_PIN, False)

    def wait_for_echo(self, value, timeout):
        count = timeout
        while GPIO.input(self.ECHO_PIN) != value and count > 0:
            count = count` - 1

    def get_distance(self):
        self.send_trigger_pulse()
        self.wait_for_echo(True, 5000)
        start = time.time()
        self.wait_for_echo(False, 5000)
        finish = time.time()
        pulse_len = finish - start
        distance_cm = pulse_len * 340 *100 /2
        distance_in = distance_cm / 2.5
        return (distance_cm, distance_in)

    def guide(self, color, station):
        path = self.path_dict[color][station]
        self.start_guide(path)
