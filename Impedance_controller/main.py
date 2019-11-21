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
        self.prev_time = rospy.get_time()
        self.prev_state = np.zeros(self.NJoints)
        self.prev_vel = np.zeros(self.NJoints)
        self.prev_vel1 = np.zeros(self.NJoints)
        self.prev_vel2 = np.zeros(self.NJoints)
        self.prev_vel3 = np.zeros(self.NJoints)
        self.prev_vel4 = np.zeros(self.NJoints)
        self.prev_wt_vel = np.zeros(self.NJoints)

    def getJointState(self, data):
        self.state = np.asarray(data.joint_positions) 
        self.controller()

    def controller(self):

        # Parameters for impedance controller
        K = 2*np.eye(self.NJoints)
        D = 0.1*np.eye(self.NJoints)
        D[5,5] = 0.5
        D[3,3] = 0.5
        M = 0.00001*np.eye(self.NJoints)

        stateGoal = np.array([-0.8,-1.5, 0.07, -0.9, -2.07, 2.2, -0.8]) 
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

        # T_int = impedence_control(self.state, vel, acc)
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
