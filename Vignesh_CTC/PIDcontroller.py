#!/usr/bin/env python
from ambf_msgs.msg import ObjectState, ObjectCmd
import rospy
import position_control_utility2 as PU
from geometry_msgs.msg import Vector3

object_name = '/ambf/env/link1/'

# State CB
def ambf_cb(msg):
    global state_msg, cmd_msg, active
    state_msg = msg
    active = True


# Initialize TK app
PU.init()
App = PU.get_app_handle()

# Default value
state_msg = ObjectState
active = False

# Initializing ROS
rospy.init_node('ambf_pid_control_test')

sub = rospy.Subscriber(object_name + 'State', ObjectState, ambf_cb, queue_size=1)
pub = rospy.Publisher(object_name + 'Command', ObjectCmd, queue_size=1)
pub_plot = rospy.Publisher('tau_plot', Vector3, queue_size=1)

rate = rospy.Rate(1000)

K_lin = 150.0
D_lin = 20.0
ki = 2
last_delta_pos = 0
delta_pos = 0
last_pos = 0
last_vel = 0
delta_delta_pos = 0
velocity_val = 0
change_vel = 0
cur_pos = 0

T = 0
cmd_msg = ObjectCmd()
cmd_msg.enable_position_controller = False

dt = 0.001

cur_time = rospy.Time.now().to_sec()

# Main Loop
while not rospy.is_shutdown():
    App.update()
    # Determining time
    last_time = cur_time
    cur_time = rospy.Time.now().to_sec()
    dt = cur_time - last_time
    if dt < 0.001:
        dt = 0.001
    if active:

        # Read current joint position
        cur_pos_val = state_msg.joint_positions
        last_cur_pos = cur_pos
        cur_pos = cur_pos_val[0]
        last_velocity_val = velocity_val
        velocity_val = (cur_pos - last_cur_pos) / dt

        # Reading desired position and velocity from user
        cmd_pos = PU.x
        desired_vel = PU.y

        # Calculation of error and rate of change of error
        last_delta_pos = delta_pos
        delta_pos = cmd_pos - cur_pos
        # print "cmd pos ", cmd_pos
        # print "cur pos ", cur_pos
        delta_delta_pos = (delta_pos - last_delta_pos) / dt
        # print "current pos value ", cur_pos
        print "delta_pos is ", delta_pos
        T = T + dt

        # The torque equation
        force = K_lin * delta_pos + D_lin * delta_delta_pos + ki*delta_pos*T

        # Equate value of torque to the ambf ROS topic
        cmd_msg.joint_cmds = [force]

        # Publishing for plotting
        vector_msg = Vector3()
        vector_msg.x = cmd_pos
        vector_msg.y = delta_pos
        vector_msg.z = cur_pos
        pub_plot.publish(vector_msg)

        # Publish the rostopic
        pub.publish(cmd_msg)

        rate.sleep()





