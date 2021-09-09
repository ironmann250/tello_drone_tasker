from djitellopy import tello
import numpy as np
import cv2
import time

me = tello.Tello()
me.connect()
print(me.get_battery())
result = me.send_control_command("downvision 0")
if result is True:
    me.stream_on = True
me.streamon()



while True:
    img = me.get_frame_read().frame
    cv2.imshow("drone", img)
    key = cv2.waitKey(1) & 0xff
    if key == 27: # ESC
        break

me.streamoff()
time.sleep(1)
result = me.send_control_command("downvision 1")
if result is True:
    me.stream_on = True
me.streamon()

while True:
    img = me.get_frame_read().frame
    cv2.imshow("drone", img)
    key = cv2.waitKey(1) & 0xff
    if key == 27: # ESC
        break