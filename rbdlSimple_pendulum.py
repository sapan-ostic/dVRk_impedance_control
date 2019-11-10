import numpy as np
import rbdl
import yaml


# Create a body for a given mass, center of mass, and inertia at
# the CoM
mass_arr = [[1.0]]
mass_arr = np.array(mass_arr) # 8x3

com_pos = [[0.5,0,0]]
com_pos = np.array(com_pos)

inertia = [[0.01,0.01,0.01]]
inertia = np.array(inertia) # 8x3


# Create a joint from joint type The revolute joint can be selected(X,Y,Z)
# can be selected to define the axis of rotation
joint_rot_z = rbdl.Joint.fromJointType ("JointTypeRevoluteZ")
# Function to get type of joint
# J_type = [['JointTypeFixed']]
# J_type = [['JointTypeRevoluteZ']] 
# J_type = np.array(J_type)

# Get the distance vector between two adjacent bodies
parent_dist = [[0,0,0],[1,0,0]]
parent_dist = np.array(parent_dist)

print 'mass', np.shape(mass_arr)
print 'inertia', np.shape(inertia)
print 'com_pos', np.shape(com_pos)
# print 'J_type', J_type
print 'parent_dist', np.shape(parent_dist)

model = rbdl.Model()

# for i in range(Num_Bodies):

    # Creating of the transformation matrix between two adjacent bodies
trans = rbdl.SpatialTransform()
trans.E = np.eye(3)
trans.r = parent_dist[0]
trans1 = rbdl.SpatialTransform()
trans1.E = np.eye(3)
trans1.r = parent_dist[1]
# Using principal inertia values from yaml file
I_x = inertia[0][0]
I_y = inertia[0][1]
I_z = inertia[0][2]

# Creation of inertia Matrix
inertia_matrix = np.array([[I_x, 0, 0], [0, I_y, 0], [0, 0, I_z]])

# Creating each body of the robot
body = rbdl.Body.fromMassComInertia(mass_arr[0], com_pos[0], inertia_matrix)

# Specifying joint Type
# joint_type = rbdl.Joint.fromJointType(joint_rot_z)

# Adding body to the model to create the complete robot
body_1 = model.AppendBody(trans, joint_rot_z, body)
body_2 = model.AppendBody(trans1, joint_rot_z, body)

print 'joint_type', joint_rot_z
print 'model', model
print 'body', body
print 'Inertia', body.mInertia
print 'trans', trans

q = np.zeros (model.q_size)
qdot = np.zeros (model.qdot_size)
qddot = np.zeros (model.qdot_size)
tau = np.zeros (model.qdot_size)

q[0] = 3.14 #-3.141592654/2
q[1] = 0 #
# Giving an arbitrary location described in the local frame and printing it's
# location wrt the world frame
COM_L1 = np.array([0, 0, 01])
COM_L1_base = rbdl.CalcBodyToBaseCoordinates (model, q, body_1, COM_L1)
print 'COM_L1_base: ', COM_L1_base
# Giving an arbitrary location described in the local frame and printing it's
# location wrt the world frame
COM_L2 = np.array([0, 0, 1])
COM_L2_base = rbdl.CalcBodyToBaseCoordinates (model, q, body_2, COM_L2)

print 'COM_L2_base: ', COM_L2_base

rbdl.InverseDynamics(model, q, qdot, qddot, tau)

print 'G: ', tau


def get_G(q_):
    q_ = np.asarray(q_)
    # print q_
    q = np.zeros(1)
    q[0] = q_[1]
    # print q
    qdot  = np.zeros(1)
    qddot = np.zeros(1)
    tau   = np.zeros(1)   

    # RBDL inverse dynamics function
    # print 'current pos:', q*180/3.1457
    rbdl.InverseDynamics(model, q, qdot, qddot, tau)
    print tau
    return tau

# q = [0.1]*7 
# q[2]=0.5   
# # q = np.asarray(q)
# Tau = get_G(q)
# print(Tau)