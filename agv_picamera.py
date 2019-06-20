import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

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


with PiCamera() as camera:
    camera.rotation = 180
    camera.start_preview()
    camera.resolution = (320, 240)
    time.sleep(1)

    with PiRGBArray(camera) as output:
        # camera.capture(output, 'rgb', use_video_port=True)
        for foo in camera.capture_continuous(output, format="bgr", use_video_port=True):
            print('Captured {}x{} image'.format(output.array.shape[1], output.array.shape[0]))

            crop_img = output.array
            hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            low_range = lower['green']
            high_range = upper['green']
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
                    pass
                if cx < 170 and cx > 110:
                    pass
                if cx <= 110:
                    pass

            # cv2.waitKey(1)

            output.truncate(0)
            break
