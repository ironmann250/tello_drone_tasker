
import cv2


img = cv2.imread("./../missions/image_feed/follow/0.jpg")
cv2.imshow("img",img)

print(img.shape)
rows,cols,ch = img.shape
print(rows,cols,ch)
img_cut = img[rows-(rows//3):rows,:]
cv2.imshow("img_cut",img_cut)
cv2.waitKey(0)
