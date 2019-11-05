#!/usr/bin/env python
from ambf_msgs.msg import ObjectState, ObjectCmd
import rospy
import position_control_utility as PU
from ambf_subscriber7 import *
global state_msg, cmd_msg, active
from geometry_msgs.msg import Vector3

    # Taking input from user regarding which link to control
var_name = raw_input("Enter the link you want to control\n")

object_name = '/ambf/env/link' + str(var_name) + '/'
n_val = int(var_name)

# ROS Subscriber callback function
def ambf_cb(msg):
    global state_msg, cmd_msg, active
    state_msg = msg
    active = True


# Initialization of TK App
PU.init()
App = PU.get_app_handle()

# Default
state_msg = ObjectState
active = False

# Initializing ROS
rospy.init_node('ambf_ctc_control_test')

sub = rospy.Subscriber(object_name + 'State', ObjectState, ambf_cb, queue_size=1)
pub = rospy.Publisher(object_name + 'Command', ObjectCmd, queue_size=1)
pub_plot = rospy.Publisher('tau_plot', Vector3, queue_size=1)

# Define ROS rate
rate = rospy.Rate(1000)

# Initialize variables
last_e = 0
e = 0
de = 0
velocity_val = 0
change_vel = 0
cur_pos = 0
accel_value = 0
t = 0

cmd_msg = ObjectCmd()
cmd_msg.enable_position_controller = False

dt = 0.001
cur_time = rospy.Time.now().to_sec()
flag = 1
count = 1
shift = 1

# Main Function
while not rospy.is_shutdown():
    App.update()
    # Read time from ROS
    last_time = cur_time
    cur_time = rospy.Time.now().to_sec()
    count += 1
    # Calculating time difference
    dt = cur_time - last_time
    if dt < 0.001:
        dt = 0.001
    if active:
        # Reading current position value of link
        cur_pos_val = state_msg.joint_positions
        last_cur_pos = cur_pos
        cur_pos = cur_pos_val[0]
        # Saving previous velocity value
        last_velocity_val= velocity_val
        # Calculating current velocity value
        velocity_val = (cur_pos - last_cur_pos) / dt

        # The Kp and Kd gains
        Kp = PU.K_lin
        Kd = PU.D_lin

        # Read from user what the desired position and velocity are
        cmd_pos = PU.x
        desired_vel = PU.y

        # Calculation of error values and rate of change of error
        last_e = e
        e = cmd_pos - cur_pos
        de = (e - last_e) / dt

        # Calculation of acceleration value
        accel_value = Kp * e + Kd * de

        # Calculating the force value for the joint, Control Law
        # force = dy+namic_model_func(cur_pos, float(desired_vel), accel_value, n_val)
        force = gravCompensation(cur_pos, float(desired_vel), accel_value, n_val, flag)
        print('count', count)
        
        if count == 4000:
            print('======== gravCompensation ON =========')
            flag = 0

        # print "Error value is ", e
        # print "force value is ", force

        # Publish the force value onto simulation
        cmd_msg.joint_cmds = [force[1]]

        # Publish values for plotting graph
        vector_msg = Vector3()
        vector_msg.x = cmd_pos
        vector_msg.y = cur_pos
        vector_msg.z = e

        # Publish messages on ROS
        pub_plot.publish(vector_msg)
        pub.publish(cmd_msg)

        rate.sleep()





