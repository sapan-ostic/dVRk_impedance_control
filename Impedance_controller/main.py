#!/usr/bin/env python

# sigma = 0.6;
# mu = 0;

# x = -2:0.1:2;
# y = (1/(sqrt(2*pi*sigma.^2)))*(exp(-(x-mu).^2)/(2*sigma.^2));

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

from ambf_msgs.msg import ObjectState, ObjectCmd
from kuka7DOF_Spatial_Transform_case import *

import numpy as np

class KukaController:

	def __init__(self):
		rospy.init_node('kuka_gravity_compensation')
		self.sub = rospy.Subscriber('/ambf/env/base/State', ObjectState, self.getJointState, queue_size=1)
		self.pub = rospy.Publisher('/ambf/env/base/Command', ObjectCmd, queue_size=1)
		self.initialize_controller()        

	def initialize_controller(self):    
		cmd_msg = ObjectCmd()        
		cmd_msg.enable_position_controller = False
		cmd_msg.position_controller_mask = [False]        

		self.NJoints = get_joint_num()
		self.cmd_msg = cmd_msg
		self.t0 = rospy.get_time()
		self.prev_time = 0

		## Joint Space
		self.prev_state = np.zeros(self.NJoints)
		self.prev_vel = np.zeros(self.NJoints)
		self.prev_vel1 = np.zeros(self.NJoints)
		self.prev_vel2 = np.zeros(self.NJoints)
		self.prev_vel3 = np.zeros(self.NJoints)
		self.prev_vel4 = np.zeros(self.NJoints)
		self.prev_wt_vel = np.zeros(self.NJoints)

		## Task Space
		self.prev_x = np.zeros(3)
		self.prev_velX = np.zeros(3)

		# Set end-effector desired velocity here
		self.X_SPEED = np.zeros(3)
		self.X_SPEED[0] = 1 
		
		self.x_speed_local = self.X_SPEED 

	def getJointState(self, data):
		self.state = np.asarray(data.joint_positions) 
		self.t = rospy.get_time() - self.t0 #clock
		self.Task_impedance_control() 

	def generate_trajectory(self, x):
		xllim = -0.6
		xulim = 0.6

		if x[0] > xulim :
			self.x_speed_local = -self.X_SPEED
		elif x[0] < xllim :
			self.x_speed_local = self.X_SPEED

		x_des = x + self.x_speed_local * (self.dt)	
		velX_des = self.x_speed_local

		print "x_speed_local : ", self.x_speed_local  
		print "velX_des", velX_des

		return [x_des, velX_des]

	def Task_impedance_control(self):

		Kp = 0.3*np.eye(3) #Stiffness Matrix
		# Kp[1][1] = 0 
		# Kp[2][2] = 0

		Kd = 0.0001*np.eye(3) # Damping Matrix
		Md = 0.02*np.eye(3)

		self.dt = self.t - self.prev_time
		x = get_end_effector_pos(self.state)
		print "x = ", x

		traj = self.generate_trajectory(x)

		# stateGoal = np.array([-0.08,-1.5, 0.07, -0.9, -2.07, 2.2, -0.8])
		XGoal = traj[0]
		XGoal[1] = -0.3
		XGoal[2] = 0.3
		XvelGoal = traj[1]
		XaccGoal = 0*np.ones(3)

		# while loop stuff here

	
		vel = (self.state - self.prev_state)/self.dt
		wt_vel = (0.925*vel + 0.7192*self.prev_vel + 0.4108*self.prev_vel1+0.09*self.prev_vel2)/(0.4108+0.7192+0.925+0.09)
		acc = (wt_vel - self.prev_wt_vel)/self.dt

		velX = (x - self.prev_x)/self.dt
		
		errX = np.asarray(XGoal - x)
		derrX = XvelGoal - velX

		J = get_end_effector_jacobian(self.state)
		J_inv = np.linalg.pinv(J)

		Mq = get_M(self.state)

		Mx = np.dot(np.dot(np.transpose(J_inv),Mq),J_inv)

		F = np.dot(np.dot(np.linalg.inv(Md), Mx),( np.dot(Kd, derrX) + np.dot(Kp, errX) ) )

		G = get_G(self.state)
		# G = inverse_dynamics(self.state, vel, acc)
		tau = G + np.dot(np.transpose(J),F)

		self.cmd_msg.joint_cmds = [tau[0],tau[1],tau[2],tau[3],tau[4],tau[5],tau[6]] 
		self.pub.publish(self.cmd_msg)

		self.prev_time = self.t    
		self.prev_state = self.state
		self.prev_vel = vel
		self.prev_vel1 = self.prev_vel
		self.prev_vel2 = self.prev_vel1
		self.prev_vel3 = self.prev_vel2
		self.prev_vel4 = self.prev_vel3
		self.prev_wt_vel = wt_vel
		self.prev_x = x			

	def CTC_joint_controller(self):

		Kp = 1*np.eye(self.NJoints)
		Kd = 0.1*np.eye(self.NJoints)
		Kd[5,5] = 0.5*0
		Kd[3,3] = 0.5*0
		# M = 0.00001*np.eye(self.NJoints)
		
		# stateGoal = np.array([-0.08,-1.5, 0.07, -0.9, -2.07, 2.2, -0.8])
		stateGoal = np.array([0]*self.NJoints) 
		velGoal = np.zeros(self.NJoints)
		accGoal = np.zeros(self.NJoints)

		# while loop stuff here
		time = rospy.get_time()
		dt = time - self.prev_time  
		# print(1/dt)      
		self.prev_time = time    

		# x = get_end_effector_pos(self.state)
		print "q = ", self.state

		vel = (self.state - self.prev_state)/dt
		wt_vel = (0.925*vel + 0.7192*self.prev_vel + 0.4108*self.prev_vel1+0.09*self.prev_vel2)/(0.4108+0.7192+0.925+0.09)
		acc = (wt_vel - self.prev_wt_vel)/dt

		err = stateGoal - self.state 
		derr = velGoal - wt_vel
		
		acc_q =  accGoal + np.dot(Kp, err) + np.dot(Kd, derr)

		tau = inverse_dynamics(self.state, wt_vel, acc_q)

		self.cmd_msg.joint_cmds = [tau[0],tau[1],tau[2],tau[3],tau[4],tau[5],tau[6]] 
		self.pub.publish(self.cmd_msg)

		self.prev_state = self.state
		self.prev_vel = vel
		self.prev_vel1 = self.prev_vel
		self.prev_vel2 = self.prev_vel1
		self.prev_vel3 = self.prev_vel2
		self.prev_vel4 = self.prev_vel3
		self.prev_wt_vel = wt_vel



	def CTC_task_controller(self):

		# Parameters for impedance controller
		Kp = 0.00*np.eye(3)
		Kd = 0.00*np.eye(3)
		Md = 0*np.eye(3)
		
		xGoal = np.array([0.5, 0.5, 0.5]) 
		velGoal = np.zeros(3)
		accGoal = np.zeros(3)


		# while loop stuff here
		time = rospy.get_time()
		dt = time - self.prev_time  
		# print(1/dt)      
		self.prev_time = time    

		x = get_end_effector_pos(self.state)
		print "x = ", x

		velX = (x - self.prev_x)/dt
		vel_q = (self.state - self.prev_state)/dt

		errX = np.asarray(xGoal - x)
		derrX = velGoal - velX

		accX = accGoal + np.dot(Kp,errX) + np.dot(Kd, derrX)  
		
		J = get_end_effector_jacobian(self.state)
		J_dot = (J - self.prev_J)/dt
		acc_q = np.dot( np.linalg.pinv(J), accX - np.dot(J_dot,vel_q) )

		tau = inverse_dynamics(self.state, vel_q, acc_q)

		self.cmd_msg.joint_cmds = [tau[0],tau[1],tau[2],tau[3],tau[4],tau[5],tau[6]] 
		self.pub.publish(self.cmd_msg)

		self.prev_state = self.state
		self.prev_x = x

	def pointcontrol_w_GravCompensation(self):

		# Parameters for impedance controller
		K = 2*np.eye(self.NJoints)
		D = 0.1*np.eye(self.NJoints)
		D[5,5] = 0.5
		D[3,3] = 0.5
		M = 0.00001*np.eye(self.NJoints)

		stateGoal = np.array([-0.08,-1.5, 0.07, -0.9, -2.07, 2.2, -0.8]) 
		stateGoal = np.copy(np.array(stateGoal[0:self.NJoints]))
		velGoal = np.zeros(self.NJoints)
		accGoal = np.zeros(self.NJoints)

		# while loop stuff here
		time = rospy.get_time()
		dt = time - self.prev_time  
		# print(1/dt)      
		self.prev_time = time    

		vel = (self.state - self.prev_state)/dt

		wt_vel = (0.925*vel + 0.7192*self.prev_vel + 0.4108*self.prev_vel1+0.09*self.prev_vel2)/(0.4108+0.7192+0.925+0.09)
		
		acc = (wt_vel - self.prev_wt_vel)/dt

		# T_int = inverse_dynamics(self.state, vel, acc)
		T_int = get_G(self.state)

		# print "T: ", T
		# print "G: ", vel

		print "state", self.state
		# print "prev_state", self.prev_state
		# print ""

		err = np.asarray(stateGoal - self.state)
		derr = velGoal - vel
		dderr = accGoal - acc

		tau = T_int + np.dot(K,err) + np.dot(D,derr) + np.dot(M,dderr) 

		self.cmd_msg.joint_cmds = [tau[0],tau[1],tau[2],tau[3],tau[4],tau[5],tau[6]] 
		self.pub.publish(self.cmd_msg)

		self.prev_state = self.state
		self.prev_vel = vel
		self.prev_vel1 = self.prev_vel
		self.prev_vel2 = self.prev_vel1
		self.prev_vel3 = self.prev_vel2
		self.prev_vel4 = self.prev_vel3
		self.prev_wt_vel = wt_vel

if __name__ == '__main__':
	start = KukaController()
	rospy.spin()
