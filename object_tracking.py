
from djitellopy import tello
import numpy as np
import cv2
import time
debug=True
testTime=0
if not debug:
    me = tello.Tello()
    me.connect()
    print(me.get_battery())

    me.streamon()
    time.sleep(0.5)
    me.takeoff()
    me.send_rc_control(0, 0, 25, 0)
    time.sleep(1)
    me.send_rc_control(0, 0, 0, 0)
else:
    me=""

w, h = 360, 240
frameWidth,frameHeight,deadZone=w,h,50
fbRange = [2500,3000]#[6200, 6800]
pidSpeed = [0.4, 0.4, 0]
pErrorSpeed = 0
pidUd = [0.4, 0.4, 0]
pErrorUd = 0

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


def trackObj(me, info, w,h, pidSpeed, pErrorSpeed,pidUd, pErrorUd):
    area = info[1]
    x, y = info[0]
    fb = 0

    errorSpeed = x - w // 2
    errorUd = y - h // 2
    speed = pidSpeed[0] * errorSpeed + pidSpeed[1] * (errorSpeed - pErrorSpeed)
    speed = int(np.clip(speed, -10, 10))
    ud = pidUd[0] * errorUd + pidUd[1] * (errorUd - pErrorUd)
    ud = int(np.clip(ud, -20, 20))
    
    if area > fbRange[0] and area < fbRange[1]:
        fb = 0

    if area > fbRange[1]:
        fb = -20
    elif area < fbRange[0] and area > 0:
        fb = 20
    
    if x == 0:
        speed = 0
        fb=0
        ud=0
        error = 0

    #print(speed, fb)
    cv2.putText(imgContour, "LR: "+str(speed)+" FB: "+str(fb)+" UD: "+str(ud),( 5, 200), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.2,(0, 0, 255), 2)
    cv2.putText(imgContour, "err LR: "+str(errorSpeed)+"err UD: "+str(errorUd),( 5, 220), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.2,(0, 0, 255), 2)
    #time.sleep(0.5)
    if not debug:
        me.send_rc_control(speed, 0, -ud, 0)
        #pass
    return errorSpeed,errorUd

if debug:
    cap = cv2.VideoCapture(0)

now=time.time()

while True:
    if testTime !=0 and (time.time()-now >=testTime):
        if not debug:
            me.land()
        break
    if debug:
        _, img = cap.read()
        img = cv2.resize(img, (w, h))
    else:
        myFrame = me.get_frame_read().frame
        img = cv2.resize(myFrame, (w, h))
    imgContour = img.copy()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    #lower = np.array([137,80,180])#h_min,s_min,v_min
    #upper = np.array([179,255,255])#h_max,s_max,v_max
    lower = np.array([36,127,126])#h_min,s_min,v_min
    upper = np.array([108,255,255])#h_max,s_max,v_max
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
    img, info = getContours(imgCanny , imgContour)
    pErrorSpeed,pErrorFb = trackObj(me, info, w,h, pidSpeed, pErrorSpeed,pidUd,pErrorUd)
    #print("Area", info[1], "Center", info[1])
    cv2.imshow("output", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        if not debug:
            me.land()
        break
