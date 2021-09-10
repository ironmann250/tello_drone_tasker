from djitellopy import tello
import cv2
import numpy as np
import time

import keyPressModule as kp

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

thresholdPixels = 0.2  # 20% threshold pixels

senstivity = 2  # sensitivity of motion change

turnWeights = [-25, -15, 0, 15, 25]  # weights of motion direction
curve = 0  # motion curve
fspeed = 20  # forward speed


def init(tello):
    """
        initializing the line follow mission, should be called first before calling
        followLine()
    """
    print("line following initializing...")

    if tello.is_flying is False:
        raise Exception('drone is not flying, can\'t start mission')

    # move to set height above the ground
    flight_height = 20  # fly at this level above the ground

    curr_height = tello.get_height()

    go_to_height_v = 0  # velocity for going to mission flight height

    if (flight_height - curr_height) > 0:
        go_to_height_v = 15
    else:
        go_to_height_v = -15

    while True:
        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.land()

        curr_height = tello.get_height()

        print(f'flying at {curr_height}cm')

        if curr_height == flight_height:
            tello.send_rc_control(0, 0, 0, 0)
            break

        tello.send_rc_control(0, 0, go_to_height_v, 0)

    print("Reached line following height of: {} cm".format(tello.get_height()))


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

    if 500 < area < 3000:
        end_of_line = True

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
    print(senOut)

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
    print(lr)
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

    while True:
        if kp.getKey("q"):  # Allow press 'q' to land in case of emergency
            tello.land()

        if DRONECAM:
            img = tello.get_frame_read().frame

            if not FRONTCAM:
                img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow("output-0", img)

            print("got drone camera stream")

            gotStream = True
        else:
            _, img = cap.read()
            print("got web cam stream")

        if gotStream:
            img = cv2.resize(img, (w, h))

            if FRONTCAM:
                img = img_cut = img[(img.shape[0] - img.shape[0] // 4):, :, :]
                imgThres = thresholding(img)  # color image thresholding
                # cv2.imshow("Cut", img_cut) # cut image
            else:
                imgThres = thresholding(img)  # color image thresholding

            # follow line
            cx, area = getContours(imgThres, img)  # image translation
            senOut = getSensorOutput(imgThres, sensors)

            if area < 100:
                # check if reached end of line
                if isEndOfLine(img):
                    # tello.send_rc_control(0, 0, 0, 0)
                    sendCommands(tello, senOut, cx)
                    time.sleep(3)
                    print("Reached end of line!")
                    # tello.send_rc_control(0, 10, 0, 0)
                    # time.sleep(1)
                    tello.send_rc_control(0, 0, 0, 0)
                    # tello.land()

                    break  # break from while loop

            sendCommands(tello, senOut, cx)

            # visualize progress
            # cv2.imshow("output", img)
            cv2.imwrite("./image_feed/follow/" + str(imgCount) + ".jpg", img)
            imgCount = imgCount + 1
            # cv2.imshow("Thres", imgThres)
            cv2.waitKey(1)
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
            tello.streamon_front()
        else:
            tello.streamon_bottom()

        time.sleep(3)
        print(tello.get_battery())
    else:
        cap = cv2.VideoCapture(0)

    tello.send_rc_control(0, 0, 0, 0)
    tello.takeoff()
    time.sleep(5)

    init(tello)  # init line follow mission
    followLine(tello)  # start line following
    deinit(cap)  # deinitialize line follow mission
