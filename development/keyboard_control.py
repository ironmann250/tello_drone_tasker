from development import keyPressModule as kp
from djitellopy import tello
from time import sleep

kp.init()

me = tello.Tello()
me.connect()
print("battery level is: "+me.get_battery())

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

while True:
    vals = getKeyBoardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    sleep(0.05)
