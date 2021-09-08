import cv2
import numpy as np

img = cv2.imread("test-img.jpg")
grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
(thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
img_cut = img[(img.shape[0] - img.shape[0] // 4):, :, :]

cv2.imshow("img", img)
cv2.imshow("img2", grayImage)
cv2.imshow("img3", blackAndWhiteImage)
cv2.imshow("img4", img_cut)
cv2.waitKey(0)
