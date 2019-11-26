# This is a test code for testing a closed loop system in the shape of 
# a four bar mechanism. In this code the creation of the system is shown
# using closedloopconstrains and the model is tested by providing some angles
# to the model and comparing the values to see if the model is described correctly.
import numpy as np
import rbdl
model = rbdl.Model()
model.gravity = np.array([0.,-9.81,0.])
l1=2.
l2=2.
m1=2.
m2=2.

Xp   = rbdl.SpatialTransform()
Xp.r = np.array([l2,0.,0.])
Xp.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

Xs   = rbdl.SpatialTransform()
Xs.r = np.array([0.,0.,0.])
Xs.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

c1 = np.array( [1.,0.,0.])

J1 = np.array([[0.1,0.,0.],
               [0.,0.1,0.],
               [0.,0.,0.1]]) 

link1 = rbdl.Body.fromMassComInertia(m1,c1,J1)
link2 = rbdl.Body.fromMassComInertia(m1,c1,J1)
cv = np.array([0.,0.,0.])
Jv = np.array([[0.,0.,0.],
               [0.,0.,0.],
               [0.,0.,0.]]) 
virtual_body=rbdl.Body.fromMassComInertia(0,cv,Jv)


joint_rot_z = rbdl.Joint.fromJointType ("JointTypeRevoluteZ")
# AddBody(parent id, joint_frame, joint, body, body name)
# Parameters:
# parent_id:   id of the parent body
# joint_frame: the transformation from the parent frame to the origin of the joint frame (represents X_T in RBDA)
# joint:       specification for the joint that describes the connection
# body:        specification of the body itself
# body_name:   human readable name for the body (can be used to retrieve its id with GetBodyId())
idB1 = model.AddBody(0.   ,Xs, joint_rot_z, link1);
idB2 = model.AddBody(idB1, Xp, joint_rot_z, link2);
idB3 = model.AddBody(0.   ,Xs, joint_rot_z, link1);
idB4 = model.AddBody(idB3, Xp, joint_rot_z, link2);
idB5 = model.AddBody(idB4, Xp, joint_rot_z, virtual_body);
# AddLoopConstraint   (id_predecessor, id_successor,SpatialTransform &  X_predecessor,SpatialTransform &  X_successor,SpatialVector&axis,enable_stabilization = false,stabilization_param = 0.1,constraint_name = NULL)   
cs = rbdl.ConstraintSet()
cs.AddLoopConstraint(idB2, idB5, Xp, Xs,rbdl.SpatialVector(0,[0,0,0,1,0,0]), False, 0.1);
cs.AddLoopConstraint(idB2, idB5, Xp, Xs,rbdl.SpatialVector(0,[0,0,0,0,1,0]), False, 0.1);
cs.AddLoopConstraint(idB2, idB5, Xp, Xs,rbdl.SpatialVector(0,[0,0,1,0,0,0]), False, 0.1);

cs.Bind(model)

q = np.zeros (model.q_size)
qdot = np.zeros (model.qdot_size)
qddot = np.zeros (model.qdot_size)
tau = np.zeros (model.qdot_size)
print("shape of q:",np.shape(q))
print("Size of q:",np.size(q))
# Testing the 4-bar linkage in the 0 degree angle for all the joints
q[0] = 0.
q[1] = 0.
q[2] = 0.
q[3] = 0.
q[4] = 0. 
pos1 =rbdl.CalcBodyToBaseCoordinates(model, q, idB2, Xp.r);
pos2 =rbdl.CalcBodyToBaseCoordinates(model, q, idB5, Xs.r);
print 'Point Location wrt base: ', pos1
print 'Point Location wrt base: ', pos2
# Testing the 4-bar linkage in the with some non-zero angle
q[0] = np.pi
q[1] = -np.pi
q[2] = np.pi-q[0]
q[3] = -q[1]
q[4] = 0. 
pos1 =rbdl.CalcBodyToBaseCoordinates(model, q, idB2, Xp.r);
pos2 =rbdl.CalcBodyToBaseCoordinates(model, q, idB5, Xs.r);
print 'Point Location wrt base: ', pos1
print 'Point Location wrt base: ', pos2
# Testing the 4-bar linkage in the with some non-zero angle
q_val=np.pi/4
q[0] = q_val
q[1] = -(np.pi*2-2*q[0])/2
q[2] = 0
q[3] = -q[1]
q[4] = 0. 
pos1 =rbdl.CalcBodyToBaseCoordinates(model, q, idB2, Xp.r);
pos2 =rbdl.CalcBodyToBaseCoordinates(model, q, idB5, Xs.r);
print 'Point Location wrt base: ', pos1
print 'Point Location wrt base: ', pos2

