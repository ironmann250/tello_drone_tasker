from djitellopy import tello
import keyPressModule as kp
import numpy as np
import cv2
import time

# kp.init()  # initialize keyboard to take keyboard commands

pid = [0.6, 0.4, 0]
pError = 0

flightHeight = 20  # fly at this level above the ground

# initialize tello
me = tello.Tello()
me.connect()
print("battery level is {}".format(me.get_battery()))

# start stream and take off
me.streamon()
me.takeoff()


def getKeyBoardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if kp.getKey("LEFT"):
        lr = -speed
    elif kp.getKey("RIGHT"):
        lr = speed
    else:
        lr = 0

    if kp.getKey("UP"):
        fb = speed
    elif kp.getKey("DOWN"):
        fb = -speed
    else:
        fb = 0

    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed
    else:
        ud = 0

    if kp.getKey("a"):
        yv = speed
    elif kp.getKey("d"):
        yv = -speed
    else:
        yv = 0

    if kp.getKey("q"): me.land()
    if kp.getKey("e"): me.takeoff()

    return [lr, fb, ud, yv]


def speedPID(sp, pv, pError):
    """
    determine speed as target is approached
    :param sp:  set point
    :param pv: processed value
    :return: speed
    """
    error = sp - pv
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))
    pError = error
    return speed, pError


# move to set height above the ground
me.send_rc_control(0, 0, flightHeight-me.get_height(), 0)
while True:
    print(me.get_height())
    if me.get_height() == flightHeight:
        me.send_rc_control(0, 0, 0, 0)
        break
print("height is: {}".format(me.get_height()))

while True:
    img = me.get_frame_read().frame
    cv2.imshow("IMG", img)
    cv2.waitKey(1)



