import numpy as np
import cv2

w, h = 360, 240  # width and height of video frame

hsvVals = [25, 36, 172, 46, 255, 255]  # home-ground # hsv range values for yellow line

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

def getContours(img_thres, img, color=(255, 0, 255)):
    """
    :param img_thres: black and white image with target from thresholding function
    :param img: colored image
    :return:
    """
    cx = 0
    area = 0
    contours, hierachy = cv2.findContours(img_thres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
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

if __name__ == '__main__':
    img = cv2.imread("./../missions/image_feed/big/0.jpg")

    img_thres = thresholding(img)  # color image thresholding
     
    doi = 20 #how much to divide an image after threshold
    img_thres = cv2.resize(img_thres, (w, h)) #resize thres image
    img_thres = img_thres[(img_thres.shape[0] - img_thres.shape[0] // doi):, :]        
        
    # follow line
    img = cv2.resize(img, (w, h)) # resize original image
    img_part =  img[(img.shape[0] - img.shape[0] // doi):, :] 
    cx, area = getContours(img_thres, img_part)  # image translation
    img[(img.shape[0] - img.shape[0] // doi):, :]  = img_part # mark line follow on original image


    cv2.imshow("img", img)
    cv2.imshow("img part", img_part)
    cv2.imshow("thresholded", img_thres)

    cv2.waitKey(0)

    cv2.destroyAllWindows()