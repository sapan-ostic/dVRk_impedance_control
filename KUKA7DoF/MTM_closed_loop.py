import numpy as np
import rbdl
import yaml


# Create a body for a given mass, center of mass, and inertia at
# the CoM
mass_arr = [[1],[1],[0.9],[0.15],[0.15],[0.8],[0.4],[0.2],[0.2],[0.1]]#Order based on the Yaml file
mass_arr = np.array(mass_arr) 

# Get the distance vector between two adjacent bodies
# The location of the joints with respect to the previous links frame. These values are directly taken from the yaml file in Joints>parent pivot.
parent_dist = [[0.0,0.0,0.0],[0.0,0.0,0.0],[ 0.0,0.0,-0.19],[0.0,0.0,0.065],[0.101,0.0,-0.03],[-0.1,-0.28,-0.035],[-0.279,-0.0,0.0],[-0.364,-0.15,0.0],[0.0,0.0,0.0],[0.0,0.002,0.0],[-0.0,-0.039,-0.0]]
parent_dist = np.array(parent_dist)

# create the COM for the bodies
# In the yaml file the center of mass of each link is described in the local coordinate frame. However, the orientation and possibly the position 
# of that frame maybe different than the frame in which your link's joint frame is described in the RBDL. So, you have to find and define the COM w.r.t
#the frame in which you described your model in the RBDL. The best way to do this is to open your model in the Blender software and campare your local frame
# with the RBDL frame and find the corresponding values for the COM.
com_pos = [[-0.008,0.064,0.028],[0.003,0.035,-0.119],[-0.109,0.012,0.02],[0.038,0.0,-0.007],[0.0,-0.14,-0.007],[-0.188,-0.008,0.0],[0.0,-0.055,-0.054],[0.0,0.041,-0.037],[0.0,-0.031,-0.03],[0.0,0.0,-0.036]]
com_pos = np.array(com_pos)

# create the inertia for the bodies
inertia = [[0.01,0.01,0.01],[0.00917612905846,0.00935165844593 ,0.00447358060957],[0.0028003998026 ,0.0134169293445 ,0.0113575925399],[0.000104513350282,0.000324014608013,0.000373281422509],[0.00110227006662,0.00000772914692154,0.00110194245711],[0.000255941352746,0.0105760140742,0.0105499806308],[0.00154081570742,0.000741332659588,0.000993899731644],[0.000397858888643,0.000210746289995,0.00022335809041],[0.000249514260856,0.000131564033426,0.000140374423125],[6.63469295292e-05,5.6484874291e-05,2.88978434528e-05]]
inertia = np.array(inertia) # 8x3


# Create a joint from joint type The revolute joint can be selected(X,Y,Z)
# can be selected to define the axis of rotation
joint_rot_z = rbdl.Joint.fromJointType ("JointTypeRevoluteZ")
joint_fixed= rbdl.Joint.fromJointType ("JointTypeFixed") #for the base and the first link
# Function to get type of joint
# J_type = [['JointTypeFixed']]
# J_type = [['JointTypeRevoluteZ']] 
# J_type = np.array(J_type)



# print 'mass', np.shape(mass_arr)
# print 'inertia', np.shape(inertia)
# print 'com_pos', np.shape(com_pos)
# print 'J_type', J_type
# print 'parent_dist', np.shape(parent_dist)

model = rbdl.Model()
model.gravity=[0,0,-9.81] # defining the direction of the gravity in the z direction (the default direction is set in the negative Y direction) 
# for i in range(Num_Bodies):

# Creating of the transformation matrix between two adjacent bodies
trans = rbdl.SpatialTransform() #world to Top panel
trans.E = np.eye(3)##world to Top panel
trans.r = parent_dist[0]#world to Top panel
trans1 = rbdl.SpatialTransform() #Top panel outpitch shoulder
trans1.E =np.array([[0.0, 1.0, 0.0],[-1.0,0.0,0.0],[0.0,0.0,1.0]])#Top panel outpitch shoulder
trans1.r = parent_dist[1]#Top panel outpitch shoulder
trans2 = rbdl.SpatialTransform() # outpitch shoulder to ArmParallel
trans2.E = np.array([[0.0, 0.0, 1.0],[1.0,0.005,0.0],[-0.005,1.0,0.0]])# outpitch shoulder to ArmParallel
trans2.r = parent_dist[2]# outpitch shoulder to ArmParallel

trans30 = rbdl.SpatialTransform()#ArmParallel-ArmParallel1
trans30.E =np.array([[-0.0002, -1.0, 0.0],[1.0,-0.0002,0.0],[0.0,0.0,1.0]])#OK ArmParallel-ArmParallel1
trans30.r = parent_dist[3]#ArmParallel-ArmParallel1

trans40 = rbdl.SpatialTransform()#ArmParallel1-ArmFront
trans40.E =np.array([[1.0, 0.0, -0.002],[0.0,1.0,0.0],[0.002,0.0,1.0]])#OK ArmParallel1-ArmFront
trans40.r = parent_dist[4]#ArmParallel1-ArmFront
# Virtual Body transformation matrix( this is basically the arm front bottom arm joint)
trans50 = rbdl.SpatialTransform()#ArmFront-BottomArm
trans50.E =np.array([[-1.0, -0.0004, -0.002],[0.0004,-1.0,0.0],[-0.002,0.0,1.0]])#OK ArmFront-BottomArm Virtual
trans50.r = parent_dist[5]#ArmFront-BottomArm

trans3 = rbdl.SpatialTransform()#ArmParallel to BottomArm
trans3.E =np.array([[-0.0002, 1.0, 0.0],[-1.0,-0.0002,0.0],[0.0,0.0,1.0]])#ArmParallel to BottomArm
trans3.r = parent_dist[6]#ArmParallel to BottomArm
trans4 = rbdl.SpatialTransform()# BottomArm to WristPlatform
trans4.E =np.array([[1.0, 0.0, 0.0],[0.0,0.0,1.0],[0.0,-1.0,0.0]])# BottomArm to WristPlatform
trans4.r = parent_dist[7]# BottomArm to WristPlatform
trans5 = rbdl.SpatialTransform()#WristPlatform to WristPitch
trans5.E =np.array([[1.0, 0.0, 0.0],[0.0,0.0,-1.0],[0.0,+1.0,0.0]])#WristPlatform to WristPitch
trans5.r = parent_dist[8]#WristPlatform to WristPitch
trans6 = rbdl.SpatialTransform()# WristPitch to WristYaw
trans6.E =np.array([[-0.0002, 0.0, 1.0],[-1.0,0.0,-0.0002],[0.0,-1.0,0.0]])# WristPitch to WristYaw
trans6.r = parent_dist[9]# WristPitch to WristYaw
trans7 = rbdl.SpatialTransform()# WristYaw to Wrist Roll
trans7.E =np.array([[-0.0002, 0.0, -1.0],[1.0,0.009,-0.0002],[0.009,-1.0,0.0]])# WristYaw to Wrist Roll
trans7.r = parent_dist[10]# WristYaw to Wrist Roll

cv = np.array([0.,0.,0.])
Jv = np.array([[0.,0.,0.],
               [0.,0.,0.],
               [0.,0.,0.]]) 
virtual_body=rbdl.Body.fromMassComInertia(0,cv,Jv) #creating the virtual mass(works as the closing point where the closed-loop system is created)

I_x = inertia[0][0]
I_y = inertia[0][1]
I_z = inertia[0][2]
I1_x = inertia[1][0]
I1_y = inertia[1][1]
I1_z = inertia[1][2]
I2_x = inertia[2][0]
I2_y = inertia[2][1]
I2_z = inertia[2][2]

I20_x = inertia[3][0]# arm parallel1 
I20_y = inertia[3][1]# arm parallel1
I20_z = inertia[3][2]# arm parallel1

I21_x = inertia[4][0]# arm Front
I21_y = inertia[4][1]# arm Front
I21_z = inertia[4][2]# arm Front

I3_x = inertia[5][0]
I3_y = inertia[5][1]
I3_z = inertia[5][2]
I4_x = inertia[6][0]
I4_y = inertia[6][1]
I4_z = inertia[6][2]
I5_x = inertia[7][0]
I5_y = inertia[7][1]
I5_z = inertia[7][2]
I6_x = inertia[8][0]
I6_y = inertia[8][1]
I6_z = inertia[8][2]
I7_x = inertia[9][0]
I7_y = inertia[9][1]
I7_z = inertia[9][2]


# Creation of inertia Matrix
inertia_matrix=[[I_x, 0, 0], [0, I_y, 0], [0, 0, I_z]],[[I1_x, 0, 0], [0, I1_y, 0], [0, 0, I1_z]],[[I2_x, 0, 0], [0, I2_y, 0], [0, 0, I2_z]],[[I20_x, 0, 0], [0, I20_y, 0], [0, 0, I20_z]],[[I21_x, 0, 0], [0, I21_y, 0], [0, 0, I21_z]],[[I3_x, 0, 0], [0, I3_y, 0], [0, 0, I3_z]],[[I4_x, 0, 0], [0, I4_y, 0], [0, 0, I4_z]],[[I5_x, 0, 0], [0, I5_y, 0], [0, 0, I5_z]],[[I6_x, 0, 0], [0, I6_y, 0], [0, 0, I6_z]],[[I7_x, 0, 0], [0, I7_y, 0], [0, 0, I7_z]]# print 'inertia', inertia_matrix
# print 'inertia', inertia_matrix[0][0]

inertia_matrix = np.array(inertia_matrix)


# Creating each body of the robot
body = rbdl.Body.fromMassComInertia(mass_arr[0], com_pos[0], inertia_matrix[0])#TopPanel
body1 = rbdl.Body.fromMassComInertia(mass_arr[1], com_pos[1], inertia_matrix[1])#Outpitch Shoulder
body2 = rbdl.Body.fromMassComInertia(mass_arr[2], com_pos[2], inertia_matrix[2])#ArmParallel

body20 = rbdl.Body.fromMassComInertia(mass_arr[3], com_pos[3], inertia_matrix[3])#ArmParallel1 
body21 = rbdl.Body.fromMassComInertia(mass_arr[4], com_pos[4], inertia_matrix[4])#Arm Front
virtual_body=rbdl.Body.fromMassComInertia(0,cv,Jv) #creating the virtual mass(works as the closing point where the closed-loop system is created)

body3 = rbdl.Body.fromMassComInertia(mass_arr[5], com_pos[5], inertia_matrix[5])#Bottom Arm
body4 = rbdl.Body.fromMassComInertia(mass_arr[6], com_pos[6], inertia_matrix[6])#Wrist Plat
body5 = rbdl.Body.fromMassComInertia(mass_arr[7], com_pos[7], inertia_matrix[7])#Wrist Pitch
body6 = rbdl.Body.fromMassComInertia(mass_arr[8], com_pos[8], inertia_matrix[8])#wrist YAW
body7 = rbdl.Body.fromMassComInertia(mass_arr[9], com_pos[9], inertia_matrix[9])#Wrist ROll

# Adding body to the model to create the complete robot
body_1 = model.AddBody(0,trans, joint_fixed, body)# TopPanel with no parent
body_2 = model.AddBody(body_1,trans1, joint_rot_z, body1)# OutpitchShoulder with TopPanel Parent
body_3 = model.AddBody(body_2,trans2, joint_rot_z, body2)# Arm Parallel with OutPitchShoulder Parent

body_30 = model.AddBody(body_3,trans30, joint_rot_z, body20)#ArmParallel 1 with ArmParallel as Parent
body_31 = model.AddBody(body_30,trans40, joint_rot_z, body21)# ArmFront with ArmParallel1 as Parent
Body_virtual =model.AddBody(body_31,trans50, joint_rot_z, virtual_body)#VirtualBody with ArmFront as Parent

body_4 = model.AddBody(body_3,trans3, joint_rot_z, body3)#BottomArm with ArmParallel as Parent
body_5 = model.AddBody(body_4,trans4, joint_rot_z, body4)#WristPlatform with BottomArm as Parent
body_6 = model.AddBody(body_5,trans5, joint_rot_z, body5)#WristPitch with WristPlatform as Parent
body_7 = model.AddBody(body_6,trans6, joint_rot_z, body6)#WristYaw with WristPitch as Parent
body_8 = model.AddBody(body_7,trans7, joint_rot_z, body7)#WristRoll with WristYaw as Parent

# AddLoopConstraint   (id_predecessor, id_successor,SpatialTransform &  X_predecessor,SpatialTransform &  X_successor,SpatialVector&axis,enable_stabilization = false,stabilization_param = 0.1,constraint_name = NULL)   
Xp   = rbdl.SpatialTransform()
Xp.r = np.array([-0.1,0.001,0.])
Xp.E = np.array([[1.,0.,-0.0056],[0.,1.,0.],[0.0056,0.,1.]])

Xs   = rbdl.SpatialTransform()
Xs.r = np.array([0.,0.,0.])
Xs.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
cs = rbdl.ConstraintSet()

cs.AddLoopConstraint(body_4, Body_virtual, Xp, Xs,rbdl.SpatialVector(0,[0,0,0,1,0,0]), False, 0.1);
cs.AddLoopConstraint(body_4, Body_virtual, Xp, Xs,rbdl.SpatialVector(0,[0,0,0,0,1,0]), False, 0.1);
cs.AddLoopConstraint(body_4, Body_virtual, Xp, Xs,rbdl.SpatialVector(0,[0,0,1,0,0,0]), False, 0.1);
# cs.AddLoopConstraint(body_4, Body_virtual, Xp, Xs,rbdl.SpatialVector(0,[0,0,0,1,0,0]), False, 0.1);
# cs.AddLoopConstraint(body_4, Body_virtual, Xp, Xs,rbdl.SpatialVector(0,[0,0,0,0,1,0]), False, 0.1);
# cs.AddLoopConstraint(body_4, Body_virtual, Xp, Xs,rbdl.SpatialVector(0,[1,0,0,0,0,0]), False, 0.1);

# cs.AddLoopConstraint(body_4, Body_virtual, Xp, Xs,rbdl.SpatialVector(0,[1,0,0,0,1,1]), False, 0.1);

cs.Bind(model)
# print 'joint_type', joint_rot_y
# print 'model', model
# print 'body', body_1
# print 'Inertia', body.mInertia
# print 'trans', trans

q = np.zeros (model.q_size)
qdot = np.zeros (model.qdot_size)
qddot = np.zeros (model.qdot_size)
tau = np.zeros (model.qdot_size)
print("shape of q:",np.shape(q))
print("Size of q:",np.size(q))
#
# Giving an arbitrary location described in the local frame and printing it's
# location wrt the world frame
# COM_L1 = np.array([0.0, 0.0, 0.0])
# COM_L1_base = rbdl.CalcBodyToBaseCoordinates (model, q, body_1, COM_L1)
# print 'COM_L1_base: ', COM_L1_base
# Giving an arbitrary location described in the local frame and printing it's
# location wrt the world frame
q[1] =3.14/2#0#-105*3.14/180# 3.14
COM_L3 = np.array([0.0, -1.0, 0.0])
COM_L3_base = rbdl.CalcBodyToBaseCoordinates (model, q, body_3, COM_L3)


print 'Point Location wrt base: ', COM_L3_base

rbdl.InverseDynamics(model, q, qdot, qddot, tau)

print 'G: ', tau


def get_G(q_):
    q_ = np.asarray(q_)
    # print "Commanded q is :", q_[1]*180/3.1457
    q = np.zeros(10)
    q[0]=q_[0]# OutPitch Shoulder
    q[1]=q_[1]# Arm Parallel
    q[5]=q_[2]# Bottom Arm
    q[2]=q_[3]# Arm Parallel1
    q[3]=q_[4]# Arm Front
    q[4]=q_[2]  # virtual body
    q[6]=q_[5]# WristPlatform
    q[7]=q_[6]# WristPitch
    q[8]=q_[7]# WristYaw
    q[9]=q_[8]# WristRoll
    # print q
    qdot  = np.zeros(10)
    qddot = np.zeros(10)
    tau   = np.zeros(10)   
    # print "q is:    ",q*180/3.1457
    # RBDL inverse dynamics function
    # print 'current pos:', q*180/3.1457
    rbdl.InverseDynamics(model, q, qdot, qddot, tau)
    # print tau
    return tau

# q = [0.1]*7 
# q[2]=0.5   
# # q = np.asarray(q)
# Tau = get_G(q)
# print(Tau)