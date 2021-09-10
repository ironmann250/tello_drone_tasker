from djitellopy import tello
import time

import keyPressModule as kp


def init(tello):
    """
        initializing the object tracking, should be called first before calling
        trackObject()
    """
    print("object tracking initializing...")


def trackObject(tello):
    """
        initializing the object tracking, should be called after calling
        init()
    """
    print("object tracking launched...")


def deinit():
    """
        deinitializing the object tracking mission
    """
    print("object tracking deinitializing...")


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
    trackObject(tello)
    deinit()
