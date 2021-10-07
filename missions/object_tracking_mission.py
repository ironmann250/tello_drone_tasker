import sys
#use the local dji tellopy
sys.path.insert(0, './../')

from djitellopy import tello
import numpy as np
import cv2
import time
import keyPressModule as kp

 #### debug vals ###
debug=False
testTime=45


### video & control vals ###
startHeight,Herror=[80,2]
waittime=0.5
change=0
timeWaited=0
multiplier=1
w, h = [360*multiplier, 240*multiplier]
frameWidth,frameHeight,deadZone=w,h,50
fbRange = [10000*(multiplier*multiplier),20000*(multiplier*multiplier)]#[6200, 6800]
pidSpeed = [0.4, 0.4, 0]
pErrorSpeed = 0
pidUd = [0.4, 0.4, 0]
pErrorUd = 0
endTargetCount=0
endTargetLimits=10

### color thresholding ###
#lower = np.array([137,80,180])#h_min,s_min,v_min
#upper = np.array([179,255,255])#h_max,s_max,v_max
#lower = np.array([89,154,83])#blue on drone h_min,s_min,v_min
#upper = np.array([179,255,255])#blue on drone h_max,s_max,v_max
#lower = np.array([45, 95, 40])#green ball h_min,s_min,v_min
#upper = np.array([82, 240, 255])#green ball h_max,s_max,v_max
lower = np.array([0,241,0])#red h_min,s_min,v_min
upper = np.array([2,255,255])#red
#lower = np.array([36,156,48])#blue outdoors h_min,s_min,v_min
#upper = np.array([133,255,255])#blue outdoors h_max,s_max,v_max


def init(tello):
    """
        initializing the object tracking, should be called first before calling
        trackObject()
    """
    print("object tracking initializing...")
    if not debug:
        tello.takeoff()
        #go to starting height within error Herror
        while(abs(tello.get_height()-startHeight)>Herror):
            print (tello.get_height())
            if kp.getKey("q"):  # Allow press 'q' to land in case of etellorgency
                tello.land()
                break
            if tello.get_height() > startHeight:
                tello.send_rc_control(0, 0, -20, 0)
            else:
                tello.send_rc_control(0 , 0, 20, 0)
            tello.send_rc_control(0 , 0, 0, 0)
        print("Reached tracking height of: {} cm".format(tello.get_height()))        

def get_mask(img,imgHsv,lower,upper): #
    mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask = mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) 
    return mask  

def getContours(img,imgContour):
    myObjectListData = []
    myObjectListC = []
    myObjectListArea = []
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin =200# cv2.getTrackbarPos("Area", "Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            #print(len(approx))
            x , y , w, h = cv2.boundingRect(approx)
            cx = x + w // 2
            cy = y + h // 2
            area = w * h
            
            myObjectListC.append((cx, cy))
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

def nonPIDtracking(me,info,w,h,imgContour):
    global change,timeWaited,waitTime

    area = info[1]
    x, y = info[0]
    fb = 0
    sensitivity=2

    errorSpeed = x - w // 2
    errorUd = y - h // 2

    speed = int(np.clip(errorSpeed//sensitivity, -100, 100))
    ud = int(np.clip(errorUd//sensitivity, -20, 20))

    if area > fbRange[0] and area < fbRange[1]:
        fb = 0

    if area > fbRange[1]:
        fb = -20
    elif area < fbRange[0] and area > 0:
        fb = 20
    
    if (abs(errorSpeed) <= 20 and abs(errorUd) <=20):
        timeWaited=time.time()-change
        if True:#timeWaited>waitTime:
            timeWaited=0
            speed = 0
            fb=0
            ud=0
            errorUd = 0
            errorSpeed = 0
    else:
        change=time.time()
        timeWaited=0

    #print(speed, fb)
    cv2.putText(imgContour, "LR: "+str(speed)+" FB: "+str(fb)+" UD: "+str(-ud),( 5, 200), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.2,(0, 0, 255), 2)
    cv2.putText(imgContour, "err LR: "+str(errorSpeed)+" err UD: "+str(x)+" tm wtd: "+str(timeWaited),( 5, 220), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.2,(0, 0, 255), 2)
    #time.sleep(0.5)
    if not debug:
        me.send_rc_control(0, fb, -ud, speed)
    
    

def trackObj(me, info, w,h, pidSpeed, pErrorSpeed,pidUd, pErrorUd,imgContour):
    global change,timeWaited,waitTime
    area = info[1]
    x, y = info[0]
    fb = 0

    errorSpeed = x - w // 2
    errorUd = y - h // 2
    speed = pidSpeed[0] * errorSpeed + pidSpeed[1] * (errorSpeed - pErrorSpeed)
    speed = int(np.clip(speed, -100, 100))
    ud = pidUd[0] * errorUd + pidUd[1] * (errorUd - pErrorUd)
    ud = int(np.clip(ud, -20, 20))
    
    if area > fbRange[0] and area < fbRange[1]:
        fb = 0

    if area > fbRange[1]:
        fb = -20
    elif area < fbRange[0] and area > 0:
        fb = 20   

    if x == 0:
        timeWaited=time.time()-change
        if True:#timeWaited>waitTime:
            timeWaited=0
            speed = 0
            fb=0
            ud=0
            errorUd = 0
            errorSpeed = 0
    else:
        change=time.time()
        timeWaited=0

    #print(speed, fb)
    cv2.putText(imgContour, "LR: "+str(speed)+" FB: "+str(fb)+" UD: "+str(-ud),( 5, 200), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.2,(0, 0, 255), 2)
    cv2.putText(imgContour, "err LR: "+str(errorSpeed)+" err UD: "+str(x)+" tm wtd: "+str(timeWaited),( 5, 220), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.2,(0, 0, 255), 2)
    #time.sleep(0.5)
    if not debug:
        me.send_rc_control(0, fb, -ud, speed)
        #pass
    return [errorSpeed,errorUd]

def isEndMission(img):
    has_target_thres=False

    if has_target_thres:
        endTargetCount=+1

    if endTargetCount >= endTargetLimits:
        return True
    else:
        return False
    """check if there is a green ball to be followed in image"""

def trackObject(tello):
    global cap,w,h,pErrorSpeed,pErrorUd
    """
        initializing the object tracking, should be called after calling
        init()
    """
    print("object tracking launched...")
    
    
    now=time.time()
    if debug:
        cap = cv2.VideoCapture(0)

    while True:
        if testTime !=0 and (time.time()-now >=testTime):
                tello.land()
                break
        if debug:
            _, img = cap.read()
            print(len(img))
            img = cv2.resize(img, (w, h))
        else:
            myFrame = tello.get_frame_read().frame
            img = cv2.resize(myFrame, (w, h))
        imgContour = img.copy()
        imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
        
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
        img, info = getContours(imgDil , imgContour)
        #pErrorSpeed,pErrorUd = trackObj(tello, info, w,h, pidSpeed, pErrorSpeed,pidUd,pErrorUd,imgContour)
        nonPIDtracking(tello, info, w,h,imgContour)
        if isEndMission:
            break
        #print("Area", info[1], "Center", info[1])
        cv2.imshow("output", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            tello.land()
            break

    


def deinit():
    """
        deinitializing the object tracking mission
    """
    print("object tracking deinitializing...")


pErrorSpeed = 0

if __name__ == "__main__":
    if not debug:
        kp.init()  # initialize pygatello keypress module

        tello = tello.Tello()
        tello.connect()
        time.sleep(1)

        tello.streamon()
        time.sleep(3)
        
        print("battery level is {}".format(tello.get_battery()))

        tello.send_rc_control(0, 0, 0, 0)



    #tello.takeoff()
    #time.sleep(3)
    
    init(tello)
    trackObject(tello)
    deinit()
