from djitellopy import tello
import time

import keyPressModule as kp


def init(tello):
    """
        initializing the obstacle avoidance, should be called first before calling
        trackObject()
    """
    print("obstacle avoidance initializing...")

    if tello.is_flying is False:
        raise Exception('drone is not flying, can\'t start mission')

        # move to set height above the ground
    flight_height = 80  # fly at this level above the ground

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


def avoidObstacles(tello):
    """
        initializing the obstacle avoidance, should be called after calling
        init()
    """
    tello.land()
    print("obstacle avoidance launched...")


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

    #tello.takeoff()
    #time.sleep(3)

    init(tello)
    avoidObstacles(tello)
    deinit()
