#main.py
import robot
import follow_line, follow_face, gps_waypoint
import manual_control, dummy_task

tasks={
	'follow_line':follow_line,
	'follow_face':follow_face,
	'gps_waypoint':gps_waypoint,
	'manual_control':manual_control,
	'dummy_task':dummy_task,
       }

while robot.active:
	print (robot.on)
	tasks[robot.active_task].run()
