import sys

from numpy.lib.shape_base import get_array_wrap
#use the local dji tellopy
sys.path.insert(0, './../')

from djitellopy import tello
import time
import cv2
import numpy as np

import keyPressModule as kp

DRONECAM = True  # using drone or computer cam

FRONTCAM = True  # get stream from front camera

if FRONTCAM:
    w, h = 360, 240  # width and height of video frame
else:
    w, h = 321, 240  # width and height of video frame

obstacle_shapes = {
    "none": 0,
    "rectangle": 1,
    "circle": 2,
    "triangle": 3 
}


shape_area_thres = 50000 # 200000 thres for the real objects area

g_flight_height = 120 # height o find objects to avoid

senstivity = 2

approach_speed = 20

forward_speed = 20

def init(tello):
    """
        initializing the obstacle avoidance, should be called first before calling
        trackObject()
    """
    print("obstacle avoidance initializing...")

    if tello.is_flying is False:
        raise Exception('drone is not flying, can\'t start mission')

        # move to set height above the ground
    flight_height = g_flight_height  # fly at this level above the ground

    curr_height = tello.get_height()

    go_to_height_v = 0  # velocity for going to mission flight height
    diff  = flight_height - curr_height
    if (diff) > 0:
        go_to_height_v = approach_speed
        tello.move_up(diff)
    else:
        go_to_height_v = -approach_speed
        tello.move_down(-diff)

    # tello.send_rc_control(0, 0, go_to_height_v, 0)
    # while True:
    #     if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
    #         tello.land()

    #     curr_height = tello.get_height()

    #     print(f'flying at {curr_height}cm')

    #     if curr_height == flight_height:
    #         tello.send_rc_control(0, 0, 0, 0)
    #         break

    print("Reached obstacle avoidance mission height of: {} cm".format(tello.get_height()))


def thresRed(img):
    """for thresholding the red color from the color image"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
    mask = mask1 | mask2
    return mask


def getContours(imgThres, img, color=(255, 0, 255)):
    """
    :param imgThres: black and white image with target from thresholding function
    :param img: colored image
    :return:
    """
    cx = 0
    cy = 0
    area = 0
    white_to_black_ratio = -1
    contours, hierachy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        biggest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h  # area of bounding box
            
        cv2.drawContours(img, biggest, -1, color, 7)
        cv2.circle(img, (cx, cy), 10, (0, 255,), cv2.FILLED)

        # cut out the region of interest in the threshold
        roi = imgThres[y:(y+h), x:(x+w)]
        cv2.imshow("roi", roi)
        total_white_of_roi = cv2.countNonZero(roi)

        # get the  non white area inside the contour if any ie for cicle and triangle
        contour_mask = np.zeros((imgThres.shape[0], imgThres.shape[1], 1), dtype="uint8")
        cv2.fillConvexPoly(contour_mask, biggest, 255)
        ret, contour_mask = cv2.threshold(contour_mask, 127, 255, cv2.THRESH_BINARY)
        x, y, w, h = cv2.boundingRect(contour_mask)
        roi_contour_mask = contour_mask[y:(y + h), x:(x + w)]
        cv2.imshow("roi_contour_mask", roi_contour_mask)
        # total_white_inside_contour = cv2.countNonZero(roi_contour_mask)

        # black region inside region of interest
        roi_black = roi_contour_mask-roi
        black_inside_roi = cv2.countNonZero(roi_black)
        print(f"white:{total_white_of_roi}, black: {black_inside_roi}")
        cv2.imshow("black area", roi_black )

        # get ratio
        white_to_black_ratio = black_inside_roi/total_white_of_roi

    print(f"contour color: {color} center:{cx,cy}, area:{area}, white/black ratio: {white_to_black_ratio}")

    return cx, cy, area, white_to_black_ratio

def isEndMission(img):
    """check if there is a green ball to be followed in image"""


def _avoidObstacles(tello,cap=None):
    """
        initializing the obstacle avoidance, should be called after calling
        init(), shouldn't be called outside this file
    """
    print("obstacle avoidance launched...")
    
    while True: 

        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.land()

        if DRONECAM:
            img = tello.get_frame_read().frame

            if not FRONTCAM:
                img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

            print("got drone camera stream")
            cv2.imshow("camera view", img)

            gotStream = True
        else:
            _, img = cap.read()
            print("got web cam stream")

        if gotStream:         

            shape, is_avoided = avoidObstacles(tello, img)
            print(f"detected shape: {shape}")
            cv2.waitKey(1)

            tello.send_rc_control(0, forward_speed, 0, 0)   #moving forward

            if avoided_shapes["triangle"]:
                print("reached end of mission")
                tello.send_rc_control(0, 0, 0, 0)   #moving forward
                tello.land()
                break

        else:
            print("waiting stream...")


def go_through_circle(tello, imgThres, white_to_black_ratio, cx, cy, area):
    """ handles operation of going through the circle """
    print("sub mission going through circle launched...")

    global senstivity

    ud = 1
    lr = 1

    cx = cx
    cy = cy
    area =  area
    white_to_black_ratio = white_to_black_ratio

    while ud or lr: # the ratio because -1 when we can't see the circle anymore
        
        ud = 0
        lr = 0

        # turning left and right
        lr = (cx - w // 2) // senstivity
        print(f"oam lr is {lr}")
        lr = int(np.clip(lr, -25, 25))
        if lr < 2 and lr > -2:
            lr = 0
        
        #moving up and down
        ud = (cy - (h) // 2) // senstivity
        print(f"oam ud is {ud}")
        ud = int(np.clip(ud, -25, 25))
        if ud < 2 and ud > -2:
            ud = 0

        # move to center of circle
        tello.send_rc_control(lr, 0, ud, 0)

        # getting another frame
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation
    
        cv2.imshow("img",img)

    # after locating center, continue going forward at a minimal speed
    tello.move_forward(200)
    tello.send_rc_control(0, forward_speed, 0, 0)


def center_to_red(tello,cx,cy):

    ud = 1
    lr = 1

    while ud and lr:

        # getting another frame
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation
    

        ud = 0
        lr = 0
        # turning left and right
        #if not lock_lr:
        lr = (cx - w // 2) // senstivity
        print(f"ctr lr is {lr}")
        lr = int(np.clip(lr, -25, 25))
        if lr < 2 and lr > -2:
            lr = 0
        
        # moving up and down
        # if not lock_ud:
        # ud = (cy - h // 2) // senstivity
        # print(f"ctr ud is {ud}")
        # ud = int(np.clip(ud, -25, 25))
        # if ud < 2 and ud > -2:
        #     ud = 0

        tello.send_rc_control(lr, 0, ud, 0) #Notice left right is the first item in the array

     
def put_object_in_center(tello, cx, cy):
    # pid_center_object(tello,cx,cy,minError=2)

    global senstivity

    tello.send_rc_control(0, 0, 0, 0)   #stop movement 

    lock_ud = False
    lock_lr = False

    while True:

        ud = 0
        lr = 0
        # turning left and right
        #if not lock_lr:
        lr = (cx - w // 2) // senstivity
        print(f"oam lr is {lr}")
        lr = int(np.clip(lr, -25, 25))
        if lr < 5 and lr > -5:
            lr = 0
            lock_lr = True
        
        # moving up and down
        # if not lock_ud:
        # ud = (cy - h // 2) // senstivity
        # print(f"oam ud is {ud}")
        # ud = int(np.clip(ud, -25, 25))
        if ud < 5 and ud > -5:
            ud = 0
            lock_ud = True

        if lock_lr and lock_ud: #object centered
            tello.send_rc_control(0, 0, 0, 0)
            break

        tello.send_rc_control(lr, 0, ud, 0) #Notice left right is the first item in the array

        # getting another frame
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation

def pid_center_object(drone,cx,cy,minError=2):
    global printElapsed,w,h
    pidSpeed,pidUd = ([0.4, 0.4, 0],[0.4, 0.4, 0])
    pErrorSpeed, pErrorUd,minError=[0,0,5]
    printElapsed=0
    debug=False
    while (True):
        x,y=[cx,cy]
        errorSpeed = x - w // 2
        errorUd = y - h // 2
        speed = pidSpeed[0] * errorSpeed + pidSpeed[1] * (errorSpeed - pErrorSpeed)
        speed = int(np.clip(speed, -20, 20))
        ud = pidUd[0] * errorUd + pidUd[1] * (errorUd - pErrorUd)
        ud = int(np.clip(ud, -20, 20))

        #Todo: evaluate when to move forward
        if x <= minError:
            speed = 0
            fb=0#test positive val from evaluation func
            ud=0
            errorUd = 0
            errorSpeed = 0
            break
        if time.time()-printElapsed <= 0.1:
            print(f'up/down: {-ud}%, left/right: {speed}%')
            printElapsed=time.time()
        if not debug:
            drone.send_rc_control(speed, 0, ud, 0)
  
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation


avoided_shapes = { #shapes that have been avoided
    "rectangle": False,
    "circle": False,
    "triangle": False
} 

imgCount = 0 #image count


ob_ratio_thre = 0.3 #threshold for shapes

circle_area_thres = 10000

rect_area_thres = 70000

def avoidObstacles(tello,frame):

    """
        obstacle avoidance, should be called after calling
        init()
    """

    global imgCount

    print(f"avoided shapes {avoided_shapes}")

    img = cv2.resize(frame, (w, h)) #resize image

    imgThres = thresRed(img)  # color image thresholding

    # avoid obstacles
    cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation

    center_to_red(tello, cx, cy)

    shape = obstacle_shapes["none"] #get type of shape
    is_avoided = False #avoidance state

    #check if any red obstacle was detected
    if white_to_black_ratio == -1:  # no red obstacle
        return shape, is_avoided    # if no red obstacle return from here
    elif (white_to_black_ratio >= ob_ratio_thre) and (area>=circle_area_thres):  # circle or triangle, 0.5 to be on a safe side ie seeing only part of the shape
        if not avoided_shapes["rectangle"] and not avoided_shapes["circle"]  and not avoided_shapes["triangle"]:
            # this is a cirle
            go_through_circle(tello, imgThres, white_to_black_ratio, cx, cy, area)

             # by this time, we assume we have moved passed the circle
            shape = obstacle_shapes["circle"] #get trype of shape
            is_avoided = True #avoidance state
            avoided_shapes["circle"] = True

        elif avoided_shapes["rectangle"]  and not avoided_shapes["circle"] and not avoided_shapes["triangle"]:
            # this is a cirle
            go_through_circle(tello, imgThres, white_to_black_ratio, cx, cy, area)

            # by this time, we assume we have moved passed the circle
            shape = obstacle_shapes["circle"] #get trype of shape
            is_avoided = True #avoidance state
            avoided_shapes["circle"] = True

        else:
            #center drone to object
            put_object_in_center(tello, cx, cy)

            # this is a triangle
            tello.move_up(50)
            tello.move_forward(200)
            tello.move_down(50)

             # by this time, we assume we have moved passed the triangle
            shape = obstacle_shapes["triangle"] #get trype of shape
            is_avoided = True #avoidance state
            avoided_shapes["triangle"] = True

    elif (white_to_black_ratio < ob_ratio_thre) and (area>=rect_area_thres):  # rectangle detected, avoid it from the left side

        #center drone to object
        put_object_in_center(tello, cx, cy)

        #avoid object 
        tello.move_up(50)
        tello.move_forward(200)
        tello.move_down(50)

        # by this time, we assume we have moved passed the rectangle
        shape = obstacle_shapes["rectangle"] #get trype of shape
        is_avoided = True #avoidance state
        avoided_shapes["rectangle"] = True


    # visualize progress
    print(f"count in oam is {imgCount}")
    cv2.imwrite("./image_feed/obstacle/" + str(imgCount) + ".jpg", img)
    imgCount = imgCount + 1
    cv2.imshow("Thres", imgThres)
    cv2.imshow("img", img)

    return shape, is_avoided


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

    #tello.streamon_front()
    tello.streamon()
    time.sleep(3)

    print("battery level is {}!".format(tello.get_battery()))

    tello.send_rc_control(0, 0, 0, 0)

    tello.takeoff()
    time.sleep(3)

    init(tello)  
    _avoidObstacles(tello)
    deinit()
