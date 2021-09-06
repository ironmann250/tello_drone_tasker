from djitellopy import tello
import cv2
import numpy as np
import time

DRONECAM = True  # using drone or computer cam

w, h = 360, 240  # width and height of video frame
hsvVals = [27, 105, 138, 179, 255, 255]  # hsv range values for yellow line

sensors = 3  # sensors in the image

thresholdPixels = 0.2  # 20% threshold pixels

senstivity = 3  # sensitivity of motion change

turnWeights = [-25, -15, 0, 15, 25]  # weights of motion direction
curve = 0  # motion curve
fspeed = 15  # forward speed

me = ""
if DRONECAM:
    me = tello.Tello()
    me.connect()
    time.sleep(1)
    print(me.get_battery())
    me.streamon()
else:
    cap = cv2.VideoCapture(0)

me.takeoff()

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


def getContours(imgThres, img):
    """
    :param imgThres: black and white image with target from thresholding function
    :param img: colored image
    :return:
    """
    cx = 0
    contours, hierachy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        biggest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest)
        cx = x + w // 2
        cy = y + h // 2
        cv2.drawContours(img, biggest, -1, (255, 0, 255), 7)
        cv2.circle(img, (cx, cy), 10, (0, 255,), cv2.FILLED)

    return cx


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
        cv2.imshow(str(x), im)
    print(senOut)

    return senOut


def sendCommands(senOut, cx):
    """
    send commands to the tello drone

    :param senOut: sensor output
    :param cx: center of the detected yellow patch
    :return: None
    """
    # Translation
    lr = (cx - w // 2) // senstivity
    lr = int(np.clip(lr, -10, 10))

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
        me.send_rc_control(lr, fspeed, 0, curve)


def followLine(tello):
    print("line follow")
    img = ""
    while True:
        print("while loop")
        if DRONECAM:
            img = tello.get_frame_read().frame
            print("drone cam")
        else:
            _, img = cap.read()
            print("web cam")

        img = cv2.resize(img, (w, h))
        imgThres = thresholding(img)
        cx = getContours(imgThres, img)  ## image translation
        senOut = getSensorOutput(imgThres, sensors)
        sendCommands(senOut, cx)
        cv2.imshow("output", img)
        cv2.imshow("Thres", imgThres)
        cv2.waitKey(1)


if not DRONECAM:
    cap.release()

cv2.destroyAllWindows()

if __name__ == '__main__':
    followLine(me)
