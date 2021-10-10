import sys

#use the local dji tellopy
sys.path.insert(0, './../')

from djitellopy import tello
import time
import cv2
import numpy as np

import keyPressModule as kp



w, h = 360, 240  # width and height of video frame


obstacle_shapes = { #the different shapes to avoid or go through
    "none": 0,
    "rectangle": 1,
    "circle": 2,
    "triangle": 3 
}

avoided_shapes = { #shapes that have been avoided
    "rectangle": False,
    "circle": False,
    "triangle": False
} 


g_flight_height = 120 # height o find objects to avoid

senstivity = 3 # sensitivity of change in direction

forward_speed = 20 # forward speed
approach_speed = 20 # speed to approach mission flight height

# threshold values
tri_cir_area_thres = 13000
rec_area_thres = 70000
ob_ratio_thres = 0.3 #threshold for shapes

imgCount = 0 #image count


def init(tello):
    """
        initializing the obstacle avoidance, should be called first before calling
        avoid_obstacle()
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


def center_still(tello):
    """ centers without leaving current position"""
    ud = 1
    lr = 1
    while ud and  lr:
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation

        # turning left and right
        lr = (cx - w // 2) // senstivity
        print(f"oam lr is {lr}")
        lr = int(np.clip(lr, -25, 25))
        if lr < 2 and lr > -2:
            lr = 0
        
        #moving up and down
        ud = ((cy+30) - h // 2) // senstivity #offset the ud because the camera is abit pointing down
        print(f"oam ud is {ud}")
        ud = int(np.clip(ud, -25, 25))
        if ud < 2 and ud > -2:
            ud = 0

        tello.send_rc_control(lr, 0, -ud, 0)



def go_through_circle(tello):
    """ handles operation of going through the circle """
    print("sub mission going through circle launched...")

    global senstivity

    img = tello.get_frame_read().frame

    img = cv2.resize(img, (w, h)) #resize image

    imgThres = thresRed(img)  # color image thresholding

    # track center
    cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation

    prev_area = area

    while True: # as long as the area is still big,we haven't moved past the circle

        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.send_rc_control(0, 0, 0, 0)   #stop drone
            tello.land()

        if area > tri_cir_area_thres*4: # close enough to the circle
            break
        
        # getting another frame
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation
        if area > prev_area:
            prev_area = area

        # turning left and right
        lr = (cx - w // 2) // senstivity
        print(f"oam lr is {lr}")
        lr = int(np.clip(lr, -25, 25))
        if lr < 2 and lr > -2:
            lr = 0

        #moving up and down
        ud = ((cy+30) - h // 2) // senstivity #offset the ud because the camera is abit pointing down
        print(f"oam ud is {ud}")
        ud = int(np.clip(ud, -25, 25))
        if ud < 2 and ud > -2:
            ud = 0

        tello.send_rc_control(lr, 20, -ud, 0)

    # after moving past circle, stop
    tello.send_rc_control(0, 0, 0, 0)
    center_still(tello)
    tello.move_down(20)
    tello.send_rc_control(10, 0, 0, 0)
    time.sleep(0.5)
    tello.send_rc_control(0, 0, 0, 0)
    tello.move_forward(130)
  


def skip_red_object(tello):
    """ handles operation of skipping object """
    print("sub mission skipping object launched...")

    global senstivity

    area = 0

    while True: # as long as the area is still big,we haven't moved past the circle

        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.send_rc_control(0, 0, 0, 0)   #stop drone
            tello.land()

        if area > tri_cir_area_thres*4: # close enough to the circle
            break
        
        # getting another frame
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation        

        # turning left and right
        lr = (cx - w // 2) // senstivity
        print(f"oam lr is {lr}")
        lr = int(np.clip(lr, -25, 25))
        if lr < 2 and lr > -2:
            lr = 0
        
        #moving up and down
        ud = ((cy+30) - h // 2) // senstivity #offset the ud because the camera is abit pointing down
        print(f"oam ud is {ud}")
        ud = int(np.clip(ud, -25, 25))
        if ud < 2 and ud > -2:
            ud = 0

        tello.send_rc_control(lr, 20, -ud, 0)

    # after moving past circle, stop
    tello.send_rc_control(0, 0, 0, 0)
    center_still(tello)

    before_height = tello.get_height()
    count_steps = 0
    while area > tri_cir_area_thres*4:  # up while going forward
        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.send_rc_control(0, 0, 0, 0)   #stop drone
            tello.land()
        tello.move_up(20) # move up until can't see object
        count_steps += 1

        # getting another frame
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # avoid obstacles
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation

    after_height = tello.get_height()

    tello.move_forward(140) #move forward abit

    tello.move_down(count_steps*20) # go back to previous height


def avoidObstacles(tello):

    """
        obstacle avoidance, should be called after calling
        init()
    """

    global imgCount

    # getting another frame
    img = tello.get_frame_read().frame

    img = cv2.resize(img, (w, h)) #resize image

    imgThres = thresRed(img)  # color image thresholding

    # avoid obstacles
    cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation

    shape = obstacle_shapes["none"] #get type of shape


    #check if any red obstacle was detected
    if white_to_black_ratio == -1:  # no red obstacle
        return shape    # if no red obstacle return from here
    elif (white_to_black_ratio >= ob_ratio_thres) and (area>=tri_cir_area_thres):  # circle or triangle, 0.5 to be on a safe side ie seeing only part of the shape
        if not avoided_shapes["rectangle"] and not avoided_shapes["circle"]  and not avoided_shapes["triangle"]:
            # this is a cirle
            go_through_circle(tello) 

             # by this time, we assume we have moved passed the circle
            shape = obstacle_shapes["circle"] #get trype of shape
            
            avoided_shapes["circle"] = True

        elif avoided_shapes["rectangle"]  and not avoided_shapes["circle"] and not avoided_shapes["triangle"]:
            # this is a cirle
            go_through_circle(tello)

            # by this time, we assume we have moved passed the circle
            shape = obstacle_shapes["circle"] #get trype of shape
            
            avoided_shapes["circle"] = True

        else:
            #center drone to object
            center_towards_red(tello, tri_cir_area_thres)

            # this is a triangle, skip it
            skip_red_object(tello)

             # by this time, we assume we have moved passed the triangle
            shape = obstacle_shapes["triangle"] #get trype of shape
            
            avoided_shapes["triangle"] = True

    elif (white_to_black_ratio < ob_ratio_thres) and (area >= tri_cir_area_thres):  # rectangle detected, avoid it from the left side

        #center drone to object
        center_towards_red(tello, rec_area_thres)

        #ski rectangle 
        skip_red_object(tello)

        # by this time, we assume we have moved passed the rectangle
        shape = obstacle_shapes["rectangle"] #get trype of shape
        
        avoided_shapes["rectangle"] = True

    # visualize progress
    print(f"count in oam is {imgCount}")
    cv2.imwrite("./image_feed/obstacle/" + str(imgCount) + ".jpg", img)
    imgCount = imgCount + 1

    print(f"avoided shapes {avoided_shapes}")

    return shape



def center_towards_red(tello, min_thres_area):
    """
    center while moving towards red, until minimum threshold area
    """

    global senstivity

    area = min_thres_area

    while area <= min_thres_area and area > 1000:
        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.send_rc_control(0, 0, 0, 0)   #stop drone
            tello.land()

        # getting another frame
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation
    
        #moving up and down
        ud = ((cy+30) - h // 2) // senstivity
        print(f"oam ud is {ud}")
        ud = int(np.clip(ud, -25, 25))
        if ud < 2 and ud > -2:
            ud = 0


        # turning left and right
        lr = (cx - w // 2) // senstivity
        print(f"ctr lr is {lr}")
        lr = int(np.clip(lr, -25, 25))
        if lr < 2 and lr > -2:
            lr = 0

        tello.send_rc_control(lr, forward_speed, -ud, 0)
        print(f"center_towards_red--->ud: {ud}, lr: {lr}, cx: {cx}, cy: {cy}, area: {area}, white_to_black_ratio: {white_to_black_ratio}")

    tello.send_rc_control(0, 0, 0, 0) 
     

def find_and_avoid(tello):
    """
        finding objects, and then calling avoiding function
    """
    print("obstacle avoidance launched...")
    
    while True: 

        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.send_rc_control(0, 0, 0, 0)   #stop drone
            tello.land()
 
        center_towards_red(tello,tri_cir_area_thres) # center while moving towards red, until minimum threshold area
        shape = avoidObstacles(tello) # take action on object

        if avoided_shapes["triangle"]:
            print("reached end of mission")
            tello.send_rc_control(0, 0, 0, 0)   #stop drone
            tello.land()
            break


def deinit():
    """
        deinitializing the obstacle avoidance mission
    """
    print("obstacle avoidance deinitializing...")


if __name__ == "__main__":
    kp.init()  # initialize pygame keypress module

    tello = tello.Tello() # initialize tello 
    tello.connect()

    tello.streamon()  #start video stream
    img = tello.get_frame_read().frame # test get an image frame before takeoff
    cv2.imshow("initial frame", img)

    print("battery level is {}!".format(tello.get_battery()))

    tello.takeoff() # drone take off

    init(tello)  # mission initialization
    find_and_avoid(tello) #
    deinit() # mission deinitialization
