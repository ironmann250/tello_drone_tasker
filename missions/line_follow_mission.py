import sys
#use the local dji tellopy
sys.path.insert(0, './../')

from djitellopy import tello
import cv2
import numpy as np
import copy
import time

import keyPressModule as kp
import obstacle_avoidance_mission as am

DRONECAM = True  # using drone or computer cam

FRONTCAM = True  # get stream from front camera

if FRONTCAM:
    w, h = 360, 240  # width and height of video frame
else:
    w, h = 321, 240  # width and height of video frame

# hsvVals = [20, 100, 100, 30, 255, 255] # base-line # yellow thres in hsv
# hsvVals = [26, 61, 137, 255, 255, 255]  # test-ground # hsv range values for yellow line
hsvVals = [25, 36, 172, 46, 255, 255]  # home-ground # hsv range values for yellow line

thres_vals = [181, 191, 188, 255, 255, 255]  # threshold range values for white line (black and white )

sensors = 3  # sensors in the image

thresholdPixels = 0.07  # 5% threshold pixels

senstivity = 2  # sensitivity of motion change

turnWeights = [-25, -15, 0, 15, 25]  # weights of motion direction
curve = 0  # motion curve
fspeed = 20  # forward speed

# offset_distance = 30 # distance in cm of drone from start of line

# frame_array = []

g_flight_height = 20

g_doi = 2

approach_speed = 20

def init(tello):
    """
        initializing the line follow mission, should be called first before calling
        followLine()
    """
    print("line following initializing...")

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


    print("Reached line following height of: {} cm".format(tello.get_height()))
    
    # approach_speed = 10
    # tello.send_rc_control(0, approach_speed, 0, 0)   # go to beginning of line at 10cm/s
    
    # t_end = time.time() + offset_distance/approach_speed
    # count = 0
    # while time.time() < t_end: #loop for set second while saving frame

    #     frame = tello.get_frame_read().frame
    #     frame_array.append(frame) #save video frame to array
    #     time.sleep(0.5)
    #     # cv2.imshow("frame",frame)
    #     # cv2.waitKey(1)
    # print(f"length of array frame is {len(frame_array)}")



def thresholding(img):
    """
    thresholding function, find the target yellow patch from the colored imaga
    :param img: colored image
    :return: mask, thresholded image
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([hsvVals[0], hsvVals[1], hsvVals[2]])
    upper = np.array([hsvVals[3], hsvVals[4], hsvVals[5]])
    mask = cv2.inRange(hsv, lower, upper)
    return mask


def thresRed(img):
    """for thresholding the red color from the color image"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
    mask = mask1 | mask2
    return mask


def thresholding_bw(img):
    """
    thresholding function, find the target white patch from the black and white image
    :param img: black and white image
    :return: mask, thresholded image
    """
    lower = np.array([thres_vals[0], thres_vals[1], thres_vals[2]])
    upper = np.array([thres_vals[3], thres_vals[4], thres_vals[5]])
    mask = cv2.inRange(img, lower, upper)
    return mask


def isEndOfLine(img):
    end_of_line = False
    mask = thresRed(img)
    cx, area = getContours(mask, img, (255, 0, 0))

    print(f"red area is: {area}")

    if 100 < area < 100:
        end_of_line = True
        #end_of_line = False

    return end_of_line


def getContours(imgThres, img, color=(255, 0, 255)):
    """
    :param imgThres: black and white image with target from thresholding function
    :param img: colored image
    :return:
    """
    cx = 0
    area = 0
    contours, hierachy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        biggest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h  # area of bounding box
        cv2.drawContours(img, biggest, -1, color, 7)
        cv2.circle(img, (cx, cy), 10, (0, 255,), cv2.FILLED)

    print(f"contour color: {color} center:{cx}, area:{area}")

    return cx, area


def getSensorOutput(imgThres, sensors):
    """
    get sensor output

    :param imgThres: black and white image with target from thresholding function
    :param sensors: the 3 element sensors
    :return: senOut
    """
    imgs = np.hsplit(imgThres, sensors)
    totalPixels = imgThres.shape[1] // sensors * imgThres.shape[0]
    senOut = []
    for x, im in enumerate(imgs):
        pixelCount = cv2.countNonZero(im)
        if pixelCount > thresholdPixels * totalPixels:
            senOut.append(1)
        else:
            senOut.append(0)
        # cv2.imshow(str(x), im)
        print(f"x is {x} totalpixels: {totalPixels}, pixelcount: {pixelCount}")
    print(f" sensor output is {senOut}")

    return senOut


def sendCommands(tello, senOut, cx):
    """
    send commands to the tello drone

    :param senOut: sensor output
    :param cx: center of the detected yellow patch
    :return: None
    """
    # Translation
    lr = (cx - w // 2) // senstivity
    print(f"lr is {lr}")
    lr = int(np.clip(lr, -100, 100))

    if lr < 2 and lr > -2:
        lr = 0

    # Rotation
    if senOut == [1, 0, 0]:
        curve = turnWeights[0]
    elif senOut == [1, 1, 0]:
        curve = turnWeights[1]
    elif senOut == [0, 1, 0]:
        curve = turnWeights[2]
    elif senOut == [0, 1, 1]:
        curve = turnWeights[3]
    elif senOut == [0, 0, 1]:
        curve = turnWeights[4]

    elif senOut == [0, 0, 0]:
        curve = turnWeights[2]
    elif senOut == [1, 1, 1]:
        curve = turnWeights[2]
    elif senOut == [1, 0, 1]:
        curve = turnWeights[2]

    if DRONECAM:
        tello.send_rc_control(curve, fspeed, 0, lr)


def followLine(tello, cap=None):
    """
    Main function that controls the following of the line, call after calling init()
    :param tello:
    :return:
    """
    print("line following launched...")

    img = ""
    imgCount = 0
    count_frames = 0  # count number of frames that have passed
    gotStream = False

         
    doi = g_doi #how much to divide an image after threshold

    while True:
        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.land()

        if DRONECAM:
            img = big_img = tello.get_frame_read().frame
            # frame_array.append(frame) #save video frame to array

            # img = big_img = frame_array.pop(0) #get image that was added first to run processing on it
            frame_copy = copy.deepcopy(img) #deep copy frame for processing by object avoidance
        
            if not FRONTCAM:
                img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow("output-0", img)

            #print("got drone camera stream")

            gotStream = True
        else:
            _, img = cap.read()
            #print("got web cam stream")

        if gotStream:

            if FRONTCAM:                
                imgThres = thresholding(img)  # color image thresholding
            else:
                imgThres = thresholding(img)  # color image thresholding
            
            imgThres = cv2.resize(imgThres, (w, h)) #resize thres image
            imgThres = imgThres[(imgThres.shape[0] - imgThres.shape[0] // doi):, :]        
               
            # follow line
            img = cv2.resize(img, (w, h)) # resize original image
            img_part =  img[(img.shape[0] - img.shape[0] // doi):, :] 
            cx, area = getContours(imgThres, img_part)  # image translation
            img[(img.shape[0] - img.shape[0] // doi):, :]  = img_part # mark line follow on original image

            # if area == 0: #no yellow has been seen, move forward
            #     tello.send_rc_control(0, 15, 0, 0)
            # else: #follow yellow
            senOut = getSensorOutput(imgThres, sensors)
            sendCommands(tello, senOut, cx)           

            # visualize progress
            cv2.imshow("output", img)
            print(f"count is {imgCount}")
            cv2.imwrite("./image_feed/follow/" + str(imgCount) + ".jpg", img)
            cv2.imwrite("./image_feed/big/" + str(imgCount) + ".jpg", big_img)
            imgCount = imgCount + 1
            # cv2.imshow("Thres", imgThres)

            # shape,is_avoided = am.avoidObstacles(tello,frame_copy) #check if there are objects to avoid

            # if shape == am.obstacle_shapes["triangle"]:
            #     # found the triangle,
            #     # by this time the shape has already been avoided

            #     tello.send_rc_control(0, 0, 0, 0) #stop drone movement
            #     break # mission finished, break from while loop

            #cv2.waitKey(1)
        else:
            print("waiting stream...")


def deinit(cap=None):
    print("line following deinitializing...")

    if not DRONECAM:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':

    kp.init()  # initialize pygame keypress module

    cap = ""
    if DRONECAM:
        tello = tello.Tello()
        tello.connect()
        time.sleep(1)
        if FRONTCAM:
            tello.streamon()
        else:
            tello.streamon_bottom()

        time.sleep(3)
        print("battery level is {}!".format(tello.get_battery()))
        
        print(f"reading first video frame...")
        img = tello.get_frame_read().frame #read first video frame
        print(f"read first video frame!")
        
    else:
        cap = cv2.VideoCapture(0)

    tello.send_rc_control(0, 0, 0, 0)
    tello.takeoff()
    time.sleep(1)

    init(tello)  # init line follow mission
    followLine(tello)  # start line following
    deinit(cap)  # deinitialize line follow mission
