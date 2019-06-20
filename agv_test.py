import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

# for front
CONTROL_PIN = 17
PWM_FREQ = 50
STEP = 15
NORTH = 90

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
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

lower = dict()
upper = dict()
# define range of blue color in HSV
lower['blue'] = np.array([110,50,50])
upper['blue'] = np.array([130,255,255])

# define range of green color in HSV 
lower['green'] = np.array([50,100,100])
upper['green'] = np.array([70,255,255])

# define range of red color in HSV 
lower['red'] = np.array([-10,100,100])
upper['red'] = np.array([10,255,255])

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

def get_photo(timeout, color):
    with PiCamera() as camera:
        camera.rotation = 180
        camera.start_preview()
        camera.resolution = (320, 240)
        time.sleep(1)

        with PiRGBArray(camera) as output:
            # camera.capture(output, 'rgb', use_video_port=True)
            for foo in camera.capture_continuous(output, format="bgr", use_video_port=True):
                if time.time() <= timeout:
                    break
                crop_img = output.array
                hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                low_range = lower[color]
                high_range = upper[color]
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

                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
                    cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
                    cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
                    print(cx, cy)

                    if cx >= 170:
                        adjust("r")
                    if cx < 170 and cx > 110:
                        turn_front()
                    if cx <= 110:
                        adjust("l")

                # cv2.waitKey(1)

                output.truncate(0)

def angle_to_duty_cycle(angle=0):
    duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * angle / 180)
    return duty_cycle

def adjust(direction):
    if direction == "r":
        # turn right
        angle = NORTH + 30
    elif direction == "l":
        # turn right
        angle = NORTH - 30
    else:
        return

    dc = angle_to_duty_cycle(angle)
    pwm.ChangeDutyCycle(dc)

def turn_front():
    dc = angle_to_duty_cycle(NORTH)
    pwm.ChangeDutyCycle(dc)

def turn_angle(direction):
    t = 1.5
    if direction == "r":
        # turn right
        angle = NORTH + 30
    elif direction == "l":
        # turn right
        angle = NORTH - 30
    elif direction == "o":
        # turn opposite
        angle = NORTH + 30
        t = 4.5
    else:
        return

    dc = angle_to_duty_cycle(angle)
    pwm.ChangeDutyCycle(dc)
    time.sleep(t)

    # turn back
    turn_front()

def guide(path, color):
    turn_front()
    count = 0
    for step in path:
        if isinstance(step, int):
            # go direct
            motor_pwm.ChangeDutyCycle(40)
            t = step * 2.5
            if count > 0:
                t = t - 0.6
            timeout = time.time() + t
            get_photo(timeout, color)
        else:
            # turn
            turn_angle(step)
        count = count + 1
    turn_front()

def main():
    color = "red"
    # station = 4
    # path_dict = get_guide_path()
    # path = path_dict[color][station]
    path = [1, 'l', 2, 'r', 1]
    guide(path, color)


if __name__ == "__main__":
    main()
