import numpy as np
import cv2

import sys

#use the local dji tellopy
sys.path.insert(0, './../')

from djitellopy import tello

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
    area = 0
    white_to_black_ratio = -1
    is_circular = False
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
        contour_mask = np.zeros((thresImg.shape[0], thresImg.shape[1], 1), dtype="uint8")
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

        # # check of element in roi is circular
        # circles = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT, 1, 20,
        #                           param1=50, param2=30, minRadius=100, maxRadius=0)
        # if circles is not None:
        #     is_circular = True
        #     for i in circles[0, :]:
        #         # draw the outer circle
        #         cv2.circle(roi, (i[0], i[1]), i[2], (0, 255, 0), 2)
        #         # draw the center of the circle
        #         cv2.circle(roi, (i[0], i[1]), 2, (0, 0, 255), 3)
        #     cv2.imshow('detected circles', roi)
        # else:
        #     is_circular = False

    print(f"contour color: {color} center:{cx}, area:{area}, white/black ratio: {white_to_black_ratio}")

    return cx, area, white_to_black_ratio, is_circular


if __name__ == '__main__':
    tello = tello.Tello()
    tello.connect()
    
    print("battery level is {}!".format(tello.get_battery()))

    tello.streamon_front()

    image_tri = cv2.imread("./red_tri.png")
    image_cir = cv2.imread("./red_cir.png")
    image_rec = cv2.imread("./red_rec.png")

    image = image_rec

    # cap = cv2.VideoCapture(0)
    while True:
        # _, image = cap.read()
        image = tello.get_frame_read().frame

        thresImg = thresRed(image)

        cx, area, white_to_black_ratio, is_circular = getContours(thresImg, image)

        cv2.imshow("Original", image)
        #cv2.imshow("thresholded", thresImg)

        cv2.waitKey(1)

    cv2.destroyAllWindows()