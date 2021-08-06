#drone
from djitellopy import Tello
import time
drone = Tello()
drone.connect(False) #if false won't cause error if it can't receive telemetry, also if false asking for telemetry will result in an error

#get one global camera stream
drone.streamon()
camera = drone.get_frame_read()

#other init values and functions
active=True
active_task='manual_control'
on=0
on+=1