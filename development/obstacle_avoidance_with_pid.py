import sys

from numpy.lib.shape_base import get_array_wrap
#use the local dji tellopy
sys.path.insert(0, './../')

from djitellopy import tello
import time
import cv2
import numpy as np

import keyPressModule as kp

DRONECAM = True  # using drone or computer cam

FRONTCAM = True  # get stream from front camera

debug=False

printDelay=0.1 #delay sec btn prints
printElapsed=0

if FRONTCAM:
    w, h = 360, 240  # width and height of video frame
else:
    w, h = 321, 240  # width and height of video frame

obstacle_shapes = {
    "none": 0,
    "rectangle": 1,
    "circle": 2,
    "triangle": 3 
}

pidSpeed, pErrorSpeed= ([0.4, 0.4, 0],[0.4, 0.4, 0])
pidUd, pErrorUd,minError=[0,0,5]

def init(tello):
    """
        initializing the obstacle avoidance, should be called first before calling
        trackObject()
    """
    print("obstacle avoidance initializing...")

    if tello.is_flying is False:
        raise Exception('drone is not flying, can\'t start mission')

        # move to set height above the ground
    flight_height = 40  # fly at this level above the ground

    curr_height = tello.get_height()

    go_to_height_v = 0  # velocity for going to mission flight height
    if (flight_height - curr_height) > 0:
        go_to_height_v = 20
    else:
        go_to_height_v = -20

    while True:
        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.land()

        curr_height = tello.get_height()

        print(f'flying at {curr_height}cm')

        if curr_height == flight_height:
            tello.send_rc_control(0, 0, 0, 0)
            break

        tello.send_rc_control(0, 0, go_to_height_v, 0)

    print("Reached obstacle avoidance mission height of: {} cm".format(tello.get_height()))

def getContours(imgThres, img, color=(255, 0, 255)):
    """
    :param imgThres: black and white image with target from thresholding function
    :param img: colored image
    :return:
    """
    cx = 0
    area = 0
    contours, hierachy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        biggest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h  # area of bounding box
        cv2.drawContours(img, biggest, -1, color, 7)
        cv2.circle(img, (cx, cy), 10, (0, 255,), cv2.FILLED)

    print(f"contour color: {color} center:{cx}, area:{area}")

    return cx, area

def thresRed(img):
    """for thresholding the red color from the color image"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
    mask = mask1 | mask2
    return mask

def isEndMission(img):
    """check if there is a green ball to be followed in image"""

def track_object(drone,cx,cy,minError=2):
    global printElapsed,w,h
    pidSpeed,pidUd = ([0.4, 0.4, 0],[0.4, 0.4, 0])
    pErrorSpeed, pErrorUd,minError=[0,0,5]

    while (True):
        x,y=[cx,cy]
        errorSpeed = x - w // 2
        errorUd = y - h // 2
        speed = pidSpeed[0] * errorSpeed + pidSpeed[1] * (errorSpeed - pErrorSpeed)
        speed = int(np.clip(speed, -20, 20))
        ud = pidUd[0] * errorUd + pidUd[1] * (errorUd - pErrorUd)
        ud = int(np.clip(ud, -20, 20))

        #Todo: evaluate when to move forward
        if x <= minError:
            speed = 0
            fb=0#test positive val from evaluation func
            ud=0
            errorUd = 0
            errorSpeed = 0
            break
        if time.time()-printElapsed <= printDelay:
            print(f'up/down: {-ud}%, left/right: {speed}&')
            printElapsed=time.time()
        if not debug:
            drone.send_rc_control(speed, 0, -ud, 0)
  
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation


def center_object(drone,cx,w,h, pidSpeed, pErrorSpeed,pidUd, pErrorUd):
    global printElapsed

    x,y=cx
    errorSpeed = x - w // 2
    errorUd = y - h // 2
    speed = pidSpeed[0] * errorSpeed + pidSpeed[1] * (errorSpeed - pErrorSpeed)
    speed = int(np.clip(speed, -100, 100))
    ud = pidUd[0] * errorUd + pidUd[1] * (errorUd - pErrorUd)
    ud = int(np.clip(ud, -20, 20))

    #Todo: evaluate when to move forward
    if x <= minError:
        speed = 0
        fb=0#test positive val from evaluation func
        ud=0
        errorUd = 0
        errorSpeed = 0
    if time.time()-printElapsed <= printDelay:
        print(f'front/back :{fb}%, up/down: {-ud}%, yaw: {speed}&')
        printElapsed=time.time()
    if not debug:
        drone.send_rc_control(speed, 0, -ud, speed)
    return errorSpeed,errorUd

def _avoidObstacles(tello,cap=None):
    """
        initializing the obstacle avoidance, should be called after calling
        init(), shouldn't be called outside this file
    """
    print("obstacle avoidance launched...")
    while True:
        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.land()

        if DRONECAM:
            img = tello.get_frame_read().frame

            if not FRONTCAM:
                img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow("output-0", img)

            print("got drone camera stream")

            gotStream = True
        else:
            _, img = cap.read()
            print("got web cam stream")

        if gotStream:
            avoidObstacles(tello, img)
            cv2.waitKey(1)
        else:
            print("waiting stream...")

imgCount = 0 #image count

def avoidObstacles(tello,frame):
    """
        initializing the obstacle avoidance, should be called after calling
        init()
    """
    # print("obstacle avoidance launched...")

    # img = cv2.resize(frame, (w, h)) #resize image

    # imgThres = thresRed(img)  # color image thresholding

    # # avoid obstacles
    # cx, area = getContours(imgThres, img)  # image translation
    # senOut = getSensorOutput(cx, area)

    # shape = obstacle_shapes["none"] #get trype of shape
    # is_avoided = True #avoidance state
    
    printElapsed=time.time()
    #call track_object
    # sendCommands(tello, senOut, cx)

    # # visualize progress
    # cv2.imwrite("./image_feed/obstacle/" + str(imgCount) + ".jpg", img)
    # imgCount = imgCount + 1
    # # cv2.imshow("Thres", imgThres)

    return [0,True]

      

def deinit():
    """
        deinitializing the obstacle avoidance mission
    """
    print("obstacle avoidance deinitializing...")


if __name__ == "__main__":
    kp.init()  # initialize pygame keypress module

    tello = tello.Tello()
    tello.connect()
    time.sleep(1)

    tello.streamon_front()
    time.sleep(3)

    print("battery level is {}".format(tello.get_battery()))

    tello.send_rc_control(0, 0, 0, 0)

    tello.takeoff()
    time.sleep(3)

    init(tello)
    _avoidObstacles(tello)
    deinit()
