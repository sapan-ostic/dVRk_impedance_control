#!/usr/bin/env python
from ambf_msgs.msg import ObjectState, ObjectCmd
import rospy
import numpy as np
import position_control_utility2 as PU
from tau_calculation import dynamic_model_func
global state_msg, cmd_msg, active
from geometry_msgs.msg import Vector3
from newinversedyna import trajgenerator, new_inverse_dynamics

object_name = '/ambf/env/base/'

# State CB
def ambf_cb(msg):
    global state_msg, cmd_msg, active
    state_msg = msg
    active = True

# Define TK App
PU.init()
App = PU.get_app_handle()

# Defaults
state_msg = ObjectState
active = False

# Initialize ROS
rospy.init_node('ambf_control_test')

sub = rospy.Subscriber(object_name + 'State', ObjectState, ambf_cb, queue_size=1)
pub = rospy.Publisher(object_name + 'Command', ObjectCmd, queue_size=1)
pub_plot = rospy.Publisher('tau_plot', Vector3, queue_size=1)

rate = rospy.Rate(1000)

last_delta_pos = 0
delta_pos = 0
last_cur_pos1, last_cur_pos2,last_cur_pos3 = 0, 0, 0
last_vel = 0
delta_delta_pos = 0
velocity_val = [0, 0, 0]
change_vel = 0
cur_pos1, cur_pos2, cur_pos3 = 0, 0, 0
calc_accel_value = 0
prev_accel = 0
accel_value = 0
T = [0, 0, 0]
cmd_msg = ObjectCmd()
cmd_msg.enable_position_controller = False
link = [0, 0, 0]
dt = 0.001

cur_time = rospy.Time.now().to_sec()

t = 0
tf = 10
th1_i = [0, 0, 0]
dth1_i = [0, 0, 0]
i = 0
# Final state

# Main Loop
while not rospy.is_shutdown():
    App.update()
    last_time = cur_time
    cur_time = rospy.Time.now().to_sec()
    dt = cur_time - last_time
    if dt < 0.001:
        dt = 0.001
    if active:

        # K_lin = np.array([[PU.roll, 0, 0], [0, PU.pitch, 0], [0, 0, PU.yaw]])
        # D_lin = np.array([[PU.K_lin, 0, 0], [0, PU.D_lin, 0], [0, 0, PU.K_ang]])
        K_lin = np.array([[10, 0, 0], [0, PU.pitch, 0], [0, 0, 11]])
        D_lin = np.array([[0, 0, 0], [0,10, 0], [0, 0, 0]])

        cur_pos_val = state_msg.joint_positions
        last_cur_pos1 = cur_pos1
        cur_pos1 = cur_pos_val[0]
        velocity_val[0] = (cur_pos1 - last_cur_pos1) / dt

        last_cur_pos2 = cur_pos2
        cur_pos2 = cur_pos_val[1]
        velocity_val[1] = (cur_pos2 - last_cur_pos2) / dt

        last_cur_pos3 = cur_pos3
        cur_pos3 = cur_pos_val[2]
        velocity_val[2] = (cur_pos3 - last_cur_pos3) / dt

        th1_f = [PU.x, PU.y, PU.z]
        dth1_f = [0, 0, 0]
        link0 = trajgenerator(th1_i[0], dth1_i[0], th1_f[0], dth1_f[0], tf)
        link1 = trajgenerator(th1_i[1], dth1_i[1], th1_f[1], dth1_f[1], tf)
        link2 = trajgenerator(th1_i[2], dth1_i[2], th1_f[2], dth1_f[2], tf)

        # t = np.linspace(0, 15, num=1501)

        accel_value, desired_vel, error_val, des_theta = new_inverse_dynamics(cur_pos_val, velocity_val, t, link0, link1, link2, K_lin, D_lin)

        t = t + 0.001
        # print "time is ", t[i]
        cur_pos = np.array([float(cur_pos_val[0]), cur_pos_val[1], cur_pos_val[2]])
        des_vel = np.array([float(desired_vel[0]), float(desired_vel[1]), float(desired_vel[2])])

        a_value = np.array([float(accel_value[0]), float(accel_value[1]), float(accel_value[2])])
        theta_desired = np.array([float(des_theta[0]), float(des_theta[1]), float(des_theta[2])])
        force = dynamic_model_func(theta_desired, des_vel, a_value)

        cmd_msg.joint_cmds = force

        vector_msg = Vector3()
        vector_msg.x = float(des_theta[1])
        vector_msg.y = float(error_val[1])
        vector_msg.z = float(desired_vel[1])
        pub_plot.publish(vector_msg)

        pub.publish(cmd_msg)

        rate.sleep()





