import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera


GPIO.setwarnings(False)

class AGV(object):
    def __init__(self, *args, **kwargs):
        self.front()
        self.back()
        self.get_guide_path()
        self.get_color_bound()

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

    def get_color_bound(self):
        self.NORTH = 90
        self.lower = dict()
        self.upper = dict()
        # define range of blue color in HSV
        self.lower['blue'] = np.array([110,50,50])
        self.upper['blue'] = np.array([130,255,255])

        # define range of green color in HSV 
        self.lower['green'] = np.array([50,100,100])
        self.upper['green'] = np.array([70,255,255])

        # define range of red color in HSV 
        self.lower['red'] = np.array([-10,100,100])
        self.upper['red'] = np.array([10,255,255])

    def angle_to_duty_cycle(self, angle=0):
        duty_cycle = (0.05 * self.PWM_FREQ) + (0.19 * self.PWM_FREQ * angle / 180)
        return duty_cycle

    def redirect(color):
        with PiCamera() as camera:
            camera.rotation = 180
            camera.start_preview()
            camera.resolution = (320, 240)

            with PiRGBArray(camera) as output:
                # camera.capture(output, 'rgb', use_video_port=True)
                last_adjust = ""
                while True:
                    camera.capture(output, format="bgr")
                    crop_img = output.array
                    hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                    low_range = self.lower[color]
                    high_range = self.upper[color]
                    gray = cv2.inRange(hsv_img, low_range, high_range)
                    blur = cv2.GaussianBlur(gray, (5, 5), 0)
                    ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

                    image, contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

                    if len(contours) > 0:
                        contours = sorted(contours, key=cv2.contourArea)
                        if len(contours) > 1:
                            del contours[-1]
                        c = max(contours, key=cv2.contourArea)
                        M = cv2.moments(c)

                        try:
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                        except:
                            break
                        if cx == 159 and cy == 119:
                            if last_adjust == "l":
                                self.adjust("r")
                                last_adjust = "r"
                            elif last_adjust == "r":
                                self.adjust("l")
                                last_adjust = "l"
                            output.truncate(0)
                            continue

                        cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
                        cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
                        cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)

                        print(cx, cy)
                        if cx >= 190:
                            self.adjust("r")
                            last_adjust = "r"
                        if cx < 190 and cx > 100:
                            self.turn_front()
                            output.truncate(0)
                            print("redirect success")
                            break
                        if cx <= 100:
                            self.adjust("l")
                            last_adjust = "l"

                    # cv2.waitKey(1)
                    output.truncate(0)

    def adjust(self, direction):
        print("redirect")
        if direction == "r":
            # turn right
            angle = self.NORTH + 30
        elif direction == "l":
            # turn right
            angle = self.NORTH - 30
        else:
            return

        dc = self.angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(dc)
        self.motor_pwm.ChangeDutyCycle(15)
        time.sleep(0.5)
        self.motor_pwm.ChangeDutyCycle(0)

    def turn_front(self):
        dc = self.angle_to_duty_cycle(self.NORTH)
        self.pwm.ChangeDutyCycle(dc)

    def turn_angle(self, direction, color):
        t = 2
        if direction == "r":
            # turn right
            angle = self.NORTH + 30
            t = 1
        elif direction == "l":
            # turn right
            angle = self.NORTH - 30
        elif direction == "o":
            # turn opposite
            angle = self.NORTH + 30
            t = 4.5
        else:
            return

        dc = self.angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(dc)
        time.sleep(t)

        # turn back
        self.redirect(color)

    def start_guide(self, path, color):
        self.turn_front()
        count = 0
        for step in path:
            print("step: ", step)
            if isinstance(step, int):
                # go direct
                self.motor_pwm.ChangeDutyCycle(40)
                t = step * 3
                if count > 0:
                    t = t - 1.5
                while t > 0:
                    print("time: ", t)
                    if t > 0.5:
                        time.sleep(0.5)
                    else:
                        time.sleep(t)
                    self.motor_pwm.ChangeDutyCycle(0)
                    self.redirect(color)
                    self.motor_pwm.ChangeDutyCycle(40)
                    t = t - 0.5
            else:
                # turn
                self.turn_angle(step, color)
            count = count + 1
        self.turn_front()

    def guide(self, color, station):
        path = self.path_dict[color][station]
        print(path, color)
        self.start_guide(path, color)
