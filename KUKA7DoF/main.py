#!/usr/bin/env python

import rospy
import std_msgs.msg
from ambf_msgs.msg import ObjectState, ObjectCmd
from dynamicsLib import *
from geometry_msgs.msg import Vector3

# our global variables

state_msg = ObjectState()
active = True
pos = []

# ROS Subscriber callback function
def get_joint_values(data):
	global state_msg, active, currentState
	pos = data.joint_positions
	# print "state: ", currentState

def main():
	sub = rospy.Subscriber('/ambf/env/base/State', ObjectState, get_joint_values, queue_size=1)
	pub = rospy.Publisher('/ambf/env/base/Command', ObjectCmd, queue_size=1)
	rospy.init_node('kuka_gravity_compensation')
	rate = rospy.Rate(1000)   #1000hz

	# Kp and Kd values
	Kp = 100
	Kd = 0.3
	Ki = 0.01
	
	# calculating the time difference
	prev_time = rospy.Time.now().to_sec() - 0.06 #adding loop time for initial dt 

	cmd_msg = ObjectCmd()
	cmd_msg.enable_position_controller = False
	cmd_msg.position_controller_mask = [False]

	prev_pos = pos

	while not rospy.is_shutdown():
		#Read time from ROS
		curr_time = rospy.Time.now().to_sec()
		dt = cur_time - prev_time
		
		prev_time = curr_time

		curr_pos = pos

		qdot = (curr_pos - prev_pos)/dt

				


		# torque_gravity_comp = Inverse_dynamics_calc_func(np.zeros(7), velocity_val, accel_val)

		# torque_pid = Kd*(velocity_pid-velocity_val) + Kp*pose_diff

		# torque = 10*torque_gravity_comp + torque_pid
		# # print(" torque is ", len(torque))

		# #define header
		# Header = std_msgs.msg.Header()
		# Header.stamp = rospy.Time.now()
		# cmd_msg.header = Header
		# # cmd_msg.joint_cmds = [torque[0], torque[1], torque[2], torque[3], torque[4], torque[5], torque[6]]
		# cmd_msg.joint_cmds = [ 0, torque[1], 0, 0, 0, 0, 0]

		# # print(" torque is ", cmd_msg.joint_cmds)

		# pub.publish(cmd_msg)
		rate.sleep()

	# rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass































