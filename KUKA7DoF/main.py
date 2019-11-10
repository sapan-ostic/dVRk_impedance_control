#!/usr/bin/env python

import rospy
import std_msgs.msg
from ambf_msgs.msg import ObjectState, ObjectCmd
# from dynamicsLib import *
from kuka2DOF import *
from geometry_msgs.msg import Vector3

# our global variables

state_msg = ObjectState()
active = True
pos = []

# ROS Subscriber callback function
def get_joint_values(data):
	global state_msg, active, pos
	pos = data.joint_positions
	pos=np.asarray(pos)
	# print(pos)
	# print "state: ", currentState
	return pos



def main():
	sub = rospy.Subscriber('/ambf/env/base/State', ObjectState, get_joint_values, queue_size=1)
	pub = rospy.Publisher('/ambf/env/base/Command', ObjectCmd, queue_size=1)
	rospy.init_node('kuka_gravity_compensation')
	rate = rospy.Rate(1000)   #1000hz

	# Kp and Kd values
	Kp = 0#1
	Kd =0 #00.001
	Ki = 0.000
	
	# calculating the time difference
	prev_time = rospy.Time.now().to_sec() - 0.06 #adding loop time for initial dt 

	cmd_msg = ObjectCmd()
	cmd_msg.enable_position_controller = False
	cmd_msg.position_controller_mask = [False]

	qGoal = 0*3.1457/180 
	qdotGoal = 0

	prev_pos = pos
	errSum = 0

	while not rospy.is_shutdown():

		curr_time = rospy.Time.now().to_sec()
		dt = curr_time - prev_time
		
		prev_time = curr_time
		curr_pos = pos
		
		print 'current pos:', curr_pos[1]*3.14/180
		
		q_dot = (curr_pos - prev_pos)/dt

		# curr_pos = [0.1]*7 
		# curr_pos[1]=0.5   	

		G = get_G(curr_pos)

		err = curr_pos[1] - qGoal
		errSum += err
		tau	= -(Kp *(err) + Kd*(q_dot[1]-qdotGoal) + Ki * errSum) +G[0] # +qgoal #G[1] #5.85*
		
		print 'tau', tau
			
		#define header
		Header = std_msgs.msg.Header()
		Header.stamp = rospy.Time.now()
		cmd_msg.header = Header
		cmd_msg.joint_cmds = [ 0, tau]#, 0, 0, 0, 0, 0]

		# # print(" torque is ", cmd_msg.joint_cmds)

		pub.publish(cmd_msg)
		rate.sleep()
	# rospy.spin()
	

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass































