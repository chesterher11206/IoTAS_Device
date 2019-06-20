import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import matplotlib.pyplot as plt
import time


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
            low_range = np.array([0, 123, 100])
            high_range = np.array([5, 255, 255])
            range_img = cv2.inRange(hsv_img, low_range, high_range)
            gray = cv2.cvtColor(range_img, cv2.COLOR_BGR2GRAY)
            plt.image.imsave('~/name.png', gray)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

            image, contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)

                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
                cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
                cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
                print(cx, cy)

                if cx >= 120:
                    pass
                if cx < 120 and cx > 50:
                    pass
                if cx <= 50:
                    pass

            # cv2.waitKey(1)

            output.truncate(0)
            break
