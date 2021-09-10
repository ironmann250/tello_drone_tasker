from djitellopy import tello
import numpy as np
import cv2
import time
import keyPressModule as kp

startHeight,Herror=[25,2]
debug=False
testtime=0
waittime=0.5
change=0
timeWaited=0
multiplier=2
w, h = 360*multiplier, 240*multiplier
fratelloWidth,fratelloHeight,deadZone=w,h,50
fbRange = [20000*(multiplier*multiplier),40000*(multiplier*multiplier)]#[6200, 6800]
pidSpeed = [0.4, 0.4, 0]
pErrorSpeed = 0
pidUd = [0.4, 0.4, 0]
pErrorUd = 0

def init(tello):
    """
        initializing the object tracking, should be called first before calling
        trackObject()
    """
    print("object tracking initializing...")
    if not debug:
        tello.takeoff()
        #go to starting height within error Herror
        while(abs(tello.get_height()-startHeight)>Herror):
            print (tello.get_height())
            if kp.getKey("q"):  # Allow press 'q' to land in case of etellorgency
                tello.land()
                break
            if tello.get_height() > startHeight:
                tello.send_rc_control(0, 0, -20, 0)
            else:
                tello.send_rc_control(0 , 0, 20, 0)
        print("Reached tracking height of: {} cm".format(tello.get_height()))        

        



def trackObject(tello):
    """
        initializing the object tracking, should be called after calling
        init()
    """
    print("object tracking launched...")
    tello.land()
    tello.streamoff()


def deinit():
    """
        deinitializing the object tracking mission
    """
    print("object tracking deinitializing...")



if __name__ == "__main__":
    kp.init()  # initialize pygatello keypress module

    tello = tello.Tello()
    tello.connect()
    time.sleep(1)

    tello.streamon()
    time.sleep(3)

    print("battery level is {}".format(tello.get_battery()))

    tello.send_rc_control(0, 0, 0, 0)

    #tello.takeoff()
    #time.sleep(3)

    init(tello)
    trackObject(tello)
    deinit()
