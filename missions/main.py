from djitellopy import tello
import time
import cv2
import numpy as np

import keyPressModule as kp

import line_follow_mission as fm
import object_tracking_mission as tm
import obstacle_avoidance_mission as am


def init():
    kp.init()  # initialize pygame keypress module
    # cv2.imshow("output", np.zeros(shape=[360, 240, 3], dtype=np.uint8))  # create cv2 window
    time.sleep(3)

    # initialize me drone
    me = tello.Tello()
    me.connect()
    print("battery level is {}".format(me.get_battery()))

    # start stream
    me.streamon_front()

    # get and show test frame from drone
    img = me.get_frame_read().frame
    cv2.imshow("output", img)

    # take off
    me.send_rc_control(0, 0, 0, 0)
    me.takeoff()

    return me


if __name__ == '__main__':

    me = init()

    missions = ["follow_line", "obstacle_avoidance", "object_tracking"]
    for mission in missions:
        if mission == "follow_line":
            """
            Execute line following mission
            """
            fm.init(me)  # init line follow mission
            fm.followLine(me)  # start line following
            fm.deinit()  # deinitialize line follow mission
        elif mission == "obstacle_avoidance":
            """
            Execute obstacle avoidance mission
            """
            am.init(me)  # init obstacle avoidance mission
            am.avoidObstacles(me)  # start obstacle avoidance
            am.deinit()  # deinitialize obstacle avoidance mission
        elif mission == "object_tracking":
            """
             Execute object tracking mission
             """
            tm.init(me)  # init object tracking mission
            tm.trackObject(me)  # start object tracking
            tm.deinit()  # deinitialize object tracking mission

    # missions finished land drone
    me.send_rc_control(0, 0, 0, 0)
    me.land()
