
from djitellopy import tello
import numpy as np
import cv2
import time
debug=True
if not debug:
    me = tello.Tello()
    me.connect()
    #print(me.get_battery())

    me.streamon()
    me.takeoff()
    me.send_rc_control(0, 0, 25, 0)
    time.sleep(3.5)
    me.send_rc_control(0, 0, 0, 0)
else:
    me=""

w, h = 360, 240
frameWidth,frameHeight,deadZone=w,h,50
fbRange = [2500,3000]#[6200, 6800]
pid = [0.4, 0.4, 0]
pError = 0

def getContours(img,imgContour):
    myObjectListData = []
    myObjectListC = []
    myObjectListArea = []
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin =0# cv2.getTrackbarPos("Area", "Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            #print(len(approx))
            x , y , w, h = cv2.boundingRect(approx)
            cx = x + w // 2
            cy = y + h // 2
            area = w * h
            
            myObjectListC.append([cx, cy])
            myObjectListArea.append(area)
            myObjectListData.append([x,y,w,h,approx])
    if len(myObjectListArea) > 0:
        i = myObjectListArea.index(max(myObjectListArea))
        cv2.circle(imgContour, myObjectListC[i], 5, (0, 255, 0), cv2.FILLED)
        x,y,w,h,approx=myObjectListData[i]
        cv2.rectangle(imgContour, (x , y ), (x + w , y + h ), (0, 255, 0), 5)

        cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,
                    (0, 255, 0), 2)
        cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                    (0, 255, 0), 2)
        cv2.putText(imgContour, " " + str(int(x))+ " "+str(int(y)), (x - 20, y- 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                    (0, 255, 0), 2)

        cx = int(x + (w / 2))
        cy = int(y + (h / 2))

        if (cx <int(frameWidth/2)-deadZone):
            cv2.putText(imgContour, " GO LEFT " , (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 1)
           
        elif (cx > int(frameWidth / 2) + deadZone):
            cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 1)
         
        elif (cy < int(frameHeight / 2) - deadZone):
            cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 1)
           
        elif (cy > int(frameHeight / 2) + deadZone):
            cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1,(0, 0, 255), 1)
        
        cv2.line(imgContour, (int(frameWidth/2),int(frameHeight/2)), (cx,cy),
                (0, 0, 255), 3)

        return imgContour, [myObjectListC[i], myObjectListArea[i]]
    else:
        return imgContour,[[0,0],0]
        
    

def findFace(img):
    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_alt.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray)

    #print(faces)

    myFaceListC = []
    myFaceListArea = []

    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)
    if len(myFaceListArea) > 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]


def trackObj(me, info, w, pid, pError):
    area = info[1]
    x, y = info[0]
    fb = 0

    error = x - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))

    if area > fbRange[0] and area < fbRange[1]:
        fb = 0

    if area > fbRange[1]:
        fb = -20
    elif area < fbRange[0] and area > 0:
        fb = 20

    if x == 0:
        speed = 0
        error = 0

    #print(speed, fb)
    cv2.putText(imgContour, "Speed: "+str(speed)+" fb: "+str(fb), (20, 200), cv2.FONT_HERSHEY_COMPLEX, 1,(0, 0, 255), 1)
    #time.sleep(0.5)
    if not debug:
        me.send_rc_control(0, fb, 0, speed)
    return error

if debug:
    cap = cv2.VideoCapture(0)
while True:
    if debug:
        _, img = cap.read()
        img = cv2.resize(img, (w, h))
    else:
        myFrame = me.get_frame_read().frame
        img = cv2.resize(myFrame, (w, h))
    imgContour = img.copy()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    lower = np.array([137,80,180])#h_min,s_min,v_min
    upper = np.array([179,255,255])#h_max,s_max,v_max
    mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask = mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
 
    imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    threshold1 = 101
    threshold2 = 0
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    img, info = getContours(imgDil, imgContour)
    pError = trackObj(me, info, w, pid, pError)
    #print("Area", info[1], "Center", info[1])
    cv2.imshow("output", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        if not debug:
            me.land()
        break
