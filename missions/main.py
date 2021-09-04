from djitellopy import tello
import keyPressModule as kp
import numpy as np
import cv2
import time

kp.init()  # initialize keyboard to take keyboard commands

pid = [0.6, 0.4, 0]
pError = 0

flightHeight = 80  # fly at 80cm

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


# move to 80cm above the ground
while True:
    #vals = getKeyBoardInput()  # just in case need to emergency land, press q
    speed, pError = speedPID(flightHeight, me.get_height(), pError)
    me.send_rc_control(0, 0, speed, 0)

    absError = abs(me.get_height() - flightHeight)
    if absError > -1 and absError < 5:
        break
    print("height is: {}, absError {}".format(me.get_height(), absError))
me.send_rc_control(0, 0, 0, 0)
me.land()

