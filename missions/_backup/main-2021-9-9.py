from djitellopy import tello
import numpy as np
import time

pid = [0.6, 0.4, 0]
pError = 0

# initialize tello drone
me = tello.Tello()
me.connect()
print("battery level is {}".format(me.get_battery()))

# start stream and take off
me.streamon()
me.takeoff()

tello = ""
cap = ""


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

