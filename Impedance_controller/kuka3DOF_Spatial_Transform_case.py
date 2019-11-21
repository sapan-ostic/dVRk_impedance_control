
import numpy as np
import rbdl
import yaml


def get_joint_num():
    num = 3
    return num

NJoints = get_joint_num()

mass_arr = [[1],[1],[1],[1],[1],[1],[1],[1]]#good
mass_arr = np.array(mass_arr) 
mass_arr = np.asarray(mass_arr[0:NJoints+1])

parent_dist = [[0.0,0.0,0.0],[0.0,0.0,0.103],[0.0,0.013,0.209],[0.0,-0.194,-0.009],[0.0,-0.013,0.202],[-0.002,0.202,-0.008],[0.002,-0.052,0.204],[-0.003,-0.05,0.053]]
parent_dist = np.array(parent_dist)
parent_dist = np.asarray(parent_dist[0:NJoints+1])

com_pos = [[0.001,0,0.06],[0.0,-0.017,0.134],[0.0,-0.074,0.009],[0.0, 0.017, 0.134],[-0.001,0.081,0.008],[0.0,-0.017,0.129],[0.0,0.007,0.068],[0.006,0.0,0.015]]
com_pos = np.array(com_pos)
com_pos = np.asarray(com_pos[0:NJoints+1])

inertia = [[0.01,0.01,0.01],[0.00815814, 0.007363868, 0.003293455],[0.00812252, 0.00329668, 0.00733904],[0.008159,0.007421,0.00330],[0.0081471,0.003297,0.0073715],[0.0077265,0.006950,0.00329],[0.002983,0.003299,0.003146],[0.000651,0.0006512,0.001120]] #
inertia = np.array(inertia) # 8x3
inertia = np.asarray(inertia[0:NJoints+1])

joint_rot_z = rbdl.Joint.fromJointType ("JointTypeRevoluteZ")
joint_fixed= rbdl.Joint.fromJointType ("JointTypeFixed") #for the base and the first link

print 'mass', np.shape(mass_arr)
print 'inertia', np.shape(inertia)
print 'com_pos', np.shape(com_pos)
print 'parent_dist', np.shape(parent_dist)

model = rbdl.Model()
model.gravity=[0,0,-9.81] # defining the direction of the gravity in the z direction (the default direction is set in the negative Y direction) 

# Creating of the transformation matrix between two adjacent bodies
trans = rbdl.SpatialTransform()
trans.E = np.eye(3)
trans.r = parent_dist[0]
trans1 = rbdl.SpatialTransform()
trans1.E =np.eye(3)
trans1.r = parent_dist[1]
trans2 = rbdl.SpatialTransform()
trans2.E = np.array([[1.0, 0.0, 0.0],[0.0,0.0,-1.0],[0.0,+1.0,0.0]])
trans2.r = parent_dist[2]
trans3 = rbdl.SpatialTransform()
trans3.E =np.array([[1.0, 0.0, 0.0],[0.0,0.0,+1.0],[0.0,-1.0,0.0]])
trans3.r = parent_dist[3]
# trans4 = rbdl.SpatialTransform()
# trans4.E =np.array([[1.0, 0.0, 0.0],[0.0,0.0,+1.0],[0.0,-1.0,0.0]])
# trans4.r = parent_dist[4]
# trans5 = rbdl.SpatialTransform()
# trans5.E =np.array([[1.0, 0.0, 0.0],[0.0,0.0,-1.0],[0.0,+1.0,0.0]])
# trans5.r = parent_dist[5]
# trans6 = rbdl.SpatialTransform()
# trans6.E =np.array([[1.0, 0.0, 0.0],[0.0,0.0,-1.0],[0.0,+1.0,0.0]])
# trans6.r = parent_dist[6]
# trans7 = rbdl.SpatialTransform()
# trans7.E =np.array([[1.0, 0.0, 0.0],[0.0,0.0,+1.0],[0.0,-1.0,0.0]])
# trans7.r = parent_dist[7]

I_x = inertia[0][0]
I_y = inertia[0][1]
I_z = inertia[0][2]
I1_x = inertia[1][0]
I1_y = inertia[1][1]
I1_z = inertia[1][2]
I2_x = inertia[2][0]
I2_y = inertia[2][1]
I2_z = inertia[2][2]
I3_x = inertia[3][0]
I3_y = inertia[3][1]
I3_z = inertia[3][2]
# I4_x = inertia[4][0]
# I4_y = inertia[4][1]
# I4_z = inertia[4][2]
# I5_x = inertia[5][0]
# I5_y = inertia[5][1]
# I5_z = inertia[5][2]
# I6_x = inertia[6][0]
# I6_y = inertia[6][1]
# I6_z = inertia[6][2]
# I7_x = inertia[7][0]
# I7_y = inertia[7][1]
# I7_z = inertia[7][2]


# Creation of inertia Matrix
inertia_matrix=[[I_x, 0, 0], [0, I_y, 0], [0, 0, I_z]],[[I1_x, 0, 0], [0, I1_y, 0], [0, 0, I1_z]],[[I2_x, 0, 0], [0, I2_y, 0], [0, 0, I2_z]],[[I3_x, 0, 0], [0, I3_y, 0], [0, 0, I3_z]]
inertia_matrix = np.array(inertia_matrix)

body = rbdl.Body.fromMassComInertia(mass_arr[0], com_pos[0], inertia_matrix[0])
body1 = rbdl.Body.fromMassComInertia(mass_arr[1], com_pos[1], inertia_matrix[1])
body2 = rbdl.Body.fromMassComInertia(mass_arr[2], com_pos[2], inertia_matrix[2])
body3 = rbdl.Body.fromMassComInertia(mass_arr[3], com_pos[3], inertia_matrix[3])
# body4 = rbdl.Body.fromMassComInertia(mass_arr[4], com_pos[4], inertia_matrix[4])
# body5 = rbdl.Body.fromMassComInertia(mass_arr[5], com_pos[5], inertia_matrix[5])
# body6 = rbdl.Body.fromMassComInertia(mass_arr[6], com_pos[6], inertia_matrix[6])
# body7 = rbdl.Body.fromMassComInertia(mass_arr[7], com_pos[7], inertia_matrix[7])

# Adding body to the model to create the complete robot
body = model.AppendBody(trans, joint_fixed, body)
body_1 = model.AppendBody(trans1, joint_rot_z, body1)
body_2 = model.AppendBody(trans2, joint_rot_z, body2)
body_3 = model.AppendBody(trans3, joint_rot_z, body3)
# body_5 = model.AppendBody(trans4, joint_rot_z, body4)
# body_6 = model.AppendBody(trans5, joint_rot_z, body5)
# body_7 = model.AppendBody(trans6, joint_rot_z, body6)
# body_8 = model.AppendBody(trans7, joint_rot_z, body7)


def get_G(q_):
    q_ = np.asarray(q_)

    NJoints = get_joint_num();

    q = np.zeros(NJoints)
    
    q[0]=q_[0]
    q[1]=q_[1]
    q[2]=q_[2]

    qdot  = np.zeros(NJoints)
    qddot = np.zeros(NJoints)
    tau   = np.zeros(NJoints)   
    
    rbdl.InverseDynamics(model, q, qdot, qddot, tau)
    return tau

def impedence_control(q_,qdot_, qddot_):
    q_ = np.asarray(q_)
    
    NJoints = get_joint_num();

    q = np.zeros(NJoints)
    
    q[0] = q_[0]
    q[1] = q_[1]
    q[2] = q_[2]
    
    qdot  = qdot_
    qddot = qddot_
    tau   = np.zeros(NJoints)   

    rbdl.InverseDynamics(model, q, qdot, qddot, tau)
    return tau
