import numpy as np
import cv2

import sys

#use the local dji tellopy
sys.path.insert(0, './../')

from djitellopy import tello


w, h = 360, 240  # width and height of video frame
senstivity = 2

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

    imgCount = 0

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
        ud = (cy - h // 2) // senstivity
        print(f"oam ud is {ud}")
        ud = int(np.clip(ud, -25, 25))
        if ud < 2 and ud > -2:
            ud = 0

        tello.send_rc_control(lr, 15, -ud, 0)
        print(f"ud is {ud} lr is {lr}")

        # getting another frame
        img = tello.get_frame_read().frame

        img = cv2.resize(img, (w, h)) #resize image

        imgThres = thresRed(img)  # color image thresholding

        # track center
        cx,  cy, area, white_to_black_ratio = getContours(imgThres, img)  # image translation
        print(f"cx:{cx} cy:{cy} area: {area} white_to_black_ratio: {white_to_black_ratio}")
    
        cv2.imshow("Stream", img)
        cv2.imwrite("./../missions/image_feed/obstacle/" + str(imgCount) + ".jpg", img)
        imgCount += 1
        #cv2.imshow("thresholded", thresImg)

        cv2.waitKey(1)

    # after locating center, continue going forward at a minimal speed
    tello.send_rc_control(0, 0, 0, 0)
    tello.move_down(20) #safe area
    tello.move_forward(200)
    tello.land()

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

    print("Reached obstacle avoidance mission height of: {} cm".format(tello.get_height()))



if __name__ == '__main__':
    tello = tello.Tello()
    tello.connect()
    
    print("battery level is {}!".format(tello.get_battery()))

    tello.streamon_front()
    image = tello.get_frame_read().frame

    tello.takeoff()

    init(tello)

    # cap = cv2.VideoCapture(0)
    while True:
        # _, image = cap.read()
        image = tello.get_frame_read().frame

        thresImg = thresRed(image)

        cx,cy, area, white_to_black_ratio = getContours(thresImg, image)

        go_through_circle(tello, thresImg, white_to_black_ratio, cx, cy, area)

        cv2.imshow("Stream", image)
        #cv2.imshow("thresholded", thresImg)

        cv2.waitKey(1)

    cv2.destroyAllWindows()