#manual_control.py
import robot
import cv2, math, time

robot.drone.takeoff()

while True:
    # In reality you want to display frames in a seperate thread. Otherwise
    #  they will freeze while the drone moves.
    img = robot.camera.frame
    cv2.imshow("drone", img)

    key = cv2.waitKey(1) & 0xff
    if key == 27: # ESC
        break
    elif key == ord('w'):
        robot.drone.move_forward(30)
    elif key == ord('s'):
        robot.drone.move_back(30)
    elif key == ord('a'):
        robot.drone.move_left(30)
    elif key == ord('d'):
        robot.drone.move_right(30)
    elif key == ord('e'):
        robot.drone.rotate_clockwise(30)
    elif key == ord('q'):
        robot.drone.rotate_counter_clockwise(30)
    elif key == ord('r'):
        robot.drone.move_up(30)
    elif key == ord('f'):
        robot.drone.move_down(30)

robot.drone.land()