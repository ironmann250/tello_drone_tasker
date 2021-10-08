import sys
#use the local dji tellopy
sys.path.insert(0, './../')

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

    tello = tello.Tello() # initialize tello 
    tello.connect()

    tello.streamon()  #start video stream

    print(f"reading first video frame...")
    img = tello.get_frame_read().frame # test get an image frame before takeoff
    cv2.imshow("initial frame", img)
    print(f"read first video frame!")

    print("battery level is {}!".format(tello.get_battery()))

    tello.takeoff() # drone take off

    height = me.get_height()
    print(f"height after take off {height}")


    return tello


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
            am.find_and_avoid(me)  # start obstacle avoidance
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
