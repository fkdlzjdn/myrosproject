	#!/usr/bin/env python

	import time
	import roslib; roslib.load_manifest('ur_driver')
	import rospy
	import actionlib


	from control_msgs.msg import *
	from trajectory_msgs.msg import *
	from sensor_msgs.msg import JointState
	from std_msgs.msg import Float32
	from math import pi

	import serial
	from numpy import * 

	import matplotlib.pyplot as plt
	from math import * 
	import threading

	 
	JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
		       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

	start_position = [0, -1.40, 0.26, -1.83, -1.57, 0]   

	Q = [0, -1.40, 0.26, -1.83, -1.57, 0]                # This value will change constantly. 
							     # Q[0] is base.  Q[1] is shoulder 
							     # Q[2] is elbow. Q[3] is list 1
							     # Q[4] is list 2.Q[5] is list 3 
							     # We just use Q[1]. 


	sensor_data = 45     # A box that receives sensor data
	time.sleep(1)

	client = None


	def average5(list):                                ###################
		v = 0

		for m in list:
			v = float(v) + float(m)

		return v / len(list)			   #### data's average is calculated
	def averageFilter(list):
		w = 0
		for n in range(-1,-6,-1):
			w = float(w) + float(list[n])
		return w / int(5)			   ###################

	def callback(Pub_data):				# Pass data to the robot
		global sensor_data			# A box that receives sensor data
		Pub_data.data = round(Pub_data.data / 10 , 2)

		sensor_data = Pub_data.data

		rospy.loginfo(Pub_data.data)


	def listener():					# date received by sensor 


		rospy.Subscriber("chatter",Float32,callback)

		rospy.spin()


	print("Start !!! \n")



	def start_point():
	    global joints_pos
	    g = FollowJointTrajectoryGoal()
	    g.trajectory = JointTrajectory()
	    g.trajectory.joint_names = JOINT_NAMES
	    try:
		joint_states = rospy.wait_for_message("joint_states", JointState)
		joints_pos = joint_states.position

		g.trajectory.points = [
			JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
			JointTrajectoryPoint(positions=start_position, velocities=[0]*6, time_from_start=rospy.Duration(3.0))]


	
		client.send_goal(g)
		client.wait_for_result()
	    
	    except KeyboardInterrupt:
		client.cancel_goal()
		raise
	    except:
		raise




	def move_point():
	    global joints_pos
	    g = FollowJointTrajectoryGoal()
	    g.trajectory = JointTrajectory()
	    g.trajectory.joint_names = JOINT_NAMES
	    try:
		joint_states = rospy.wait_for_message("joint_states", JointState)
		joints_pos = joint_states.position

		g.trajectory.points = [
			JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
			JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(0.8))]


	
		client.send_goal(g)
		client.wait_for_result()
	    
	    except KeyboardInterrupt:
		client.cancel_goal()
		raise
	    except:
		raise
	    
	   
	def whilemove():
	    global joints_pos
	    g = FollowJointTrajectoryGoal()
	    g.trajectory = JointTrajectory()
	    g.trajectory.joint_names = JOINT_NAMES

	    data_box,filter_data_box = [],[]
	    global sensor_data
	    global Q
	    
	    data_cnt=0              # Sensor data count
	    max_height = -1.4		# robot's max position
	    min_height = -0.52		# robot's min position
	    min_unit = 0.02		# robot's position change rate
	    shoulder_joint = 0
	    average_set = 5
	  
	  
	    try:
		joint_states = rospy.wait_for_message("joint_states", JointState)
		joints_pos = joint_states.position



		while True :	

			data_box.append(sensor_data)
			if data_cnt > average_set:
				filter_data_box.append(averageFilter(data_box))
			else :
				filter_data_box.append(average3(data_box))
		

			shoulder_joint = min_height - (min_unit * filter_data_box[-1])  # filter_data_box[-1] is the most recent data.
	 		
			print(filter_data_box[-1])
			print(shoulder_joint)

			if (shoulder_joint > min_height ):
				shoulder_joint = float(min_height) 
			elif (shoulder_joint < max_height):

				shoulder_joint = float(max_height) 	

			 
			print(shoulder_joint)
		


			Q[1] = shoulder_joint
			move_point()
		

		
			data_cnt += 1
		

			client.send_goal(g)
			client.wait_for_result()	    

	    
	    except KeyboardInterrupt:
		client.cancel_goal()
		raise
	    except:
		raise









	def main():
	    global client
	    th_ur = threading.Thread(target=whilemove)
	    
	    try:

		rospy.init_node("UR3_move", anonymous=True, disable_signals=True)
		client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
		print "Waiting for server..."
		client.wait_for_server()
		print "Connected to server"
		parameters = rospy.get_param(None)
		index = str(parameters).find('prefix')
		if (index > 0):
		    prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
		    for i, name in enumerate(JOINT_NAMES):
			JOINT_NAMES[i] = prefix + name
		print "This program makes the robot move between the following three poses:"
		print str([Q[i]*180./pi for i in xrange(0,6)])

		print "Please make sure that your robot can move freely between these poses before proceeding!"
		inp = raw_input("Continue? y/n: ")[0]
		if (inp == 'y'):
		    
		    start_point()
		    

		    th_ur.start()
		    listener()


	
	
		else:
		    print "Halting program"
		print str([Q[i]*180./pi for i in xrange(0,6)])

	    except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

	if __name__ == '__main__': main()
