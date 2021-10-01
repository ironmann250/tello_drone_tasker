from djitellopy import tello

import cv2

import numpy as np

import time

import image_recolor as recolor

DRONECAM = True  # using drone or computer cam
ISBW = False

frameWidth = 480

frameHeight = 360

me = ""

if DRONECAM:
    me = tello.Tello()

    me.connect()
    time.sleep(1)
    print(me.get_battery())

    if ISBW:
        tello.streamon_bottom()
    else:
        tello.streamon_front()
else:
    cap = cv2.VideoCapture(0)


def empty(a):
    pass


valInit = [0, 36, 179, 81, 255, 255]

cv2.namedWindow("HSV")

cv2.resizeWindow("HSV", 640, 240)

cv2.createTrackbar("HUE Min", "HSV", valInit[0], 255, empty)

cv2.createTrackbar("HUE Max", "HSV", valInit[3], 255, empty)

cv2.createTrackbar("SAT Min", "HSV", valInit[1], 255, empty)

cv2.createTrackbar("SAT Max", "HSV", valInit[4], 255, empty)

cv2.createTrackbar("VALUE Min", "HSV", valInit[2], 255, empty)

cv2.createTrackbar("VALUE Max", "HSV", valInit[5], 255, empty)

frameCounter = 0

while True:

    if DRONECAM:
        img = me.get_frame_read().frame
    else:
        _, img = cap.read()

    img = cv2.resize(img, (frameWidth, frameHeight))

    # img = cv2.flip(img, 0)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    h_min = cv2.getTrackbarPos("HUE Min", "HSV")

    h_max = cv2.getTrackbarPos("HUE Max", "HSV")

    s_min = cv2.getTrackbarPos("SAT Min", "HSV")

    s_max = cv2.getTrackbarPos("SAT Max", "HSV")

    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")

    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

    lower = np.array([h_min, s_min, v_min])

    upper = np.array([h_max, s_max, v_max])

    if ISBW:  # black and white image
        img = recolor.colorize(img)

        imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(img, lower, upper)

        result = cv2.bitwise_and(img, img, mask=mask)
    else:
        imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(imgHsv, lower, upper)

        result = cv2.bitwise_and(img, img, mask=mask)

    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    print(f'[{h_min},{s_min},{v_min},{h_max},{s_max},{v_max}]')

    hStack = np.hstack([img, mask, result])

    cv2.imshow('Horizontal Stacking', hStack)

    if cv2.waitKey(1) and 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
