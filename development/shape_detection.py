import cv2
import numpy as np
from matplotlib import pyplot as plt
  
# reading image
cap = cv2.VideoCapture(0)
lower = np.array([0,241,0])#red h_min,s_min,v_min
upper = np.array([2,255,255])#red

def thresRed(img):
    """for thresholding the red color from the color image"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
    mask = mask1 | mask2
    return mask

while True:
    _, img = cap.read()#cv2.imread("shapes.png")
    print(len(img))
    
    #imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    #mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask = thresRed(img))
    #mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    #imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
    
    # converting image into grayscale image
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    
    # setting threshold of gray image
    #_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    threshold1 = 100
    threshold2 = 0
    imgCanny = cv2.Canny(gray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    threshold = cv2.dilate(imgCanny, kernel, iterations=1)
    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    i = 0
    
    # list for storing names of shapes
    for contour in contours:
    
        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue
    
        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.01 * cv2.arcLength(contour, True), True)
        
        # using drawContours() function
        
        x_ , y_ , w_, h_ = cv2.boundingRect(approx)
        cx = x_ + w_ // 2
        cy = y_ + h_ // 2
        area = w_ * h_
        if area>105000:
            cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)
        # finding center point of shape
        x,y=0,0
        M = cv2.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])
    
        # putting shape name at center of each shape
        if len(approx) == 3:
            cv2.putText(img, 'Triangle', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        elif len(approx) == 4:
            cv2.putText(img, 'Quadrilateral', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        elif len(approx) == 5:
            cv2.putText(img, 'Pentagon', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        elif len(approx) == 6:
            cv2.putText(img, 'Hexagon', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        else:
            cv2.putText(img, 'circle', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # displaying the image after drawing contours
    cv2.imshow('shapes', img)
    cv2.imshow('edges', threshold)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
