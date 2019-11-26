import numpy as np
import rbdl
model = rbdl.Model()
model.gravity = np.array([0.,-9.81,0.])

l1=1.
l2=1.
m1=1.
m2=1.
c1 = np.array([0.,-l1*0.5,0.])
J1 = np.array([[m1*l1*l1/3.,           0.,          0.],
               [         0., m1*l1*l1/30.,          0.],
               [         0.,           0., m1*l1*l1/3.]]) 

c2 = np.array([l2*0.5,0.,0.])        
J2 = np.array([ [m2*l2*l2/30.,          0., 0.],
                [          0., m2*l2*l2/3., 0.],
                [          0.,          0., m2*l2*l2/3.]]) 

Xp1   = rbdl.SpatialTransform()
Xp1.r = np.array([0.,0.,0.])
Xp1.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

Xs1   = rbdl.SpatialTransform()
Xs1.r = np.array([0.,0.,0.])
Xs1.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

Xp2   = rbdl.SpatialTransform()
Xp2.r = np.array([0.,-l1,0.])
Xp2.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

Xs2   = rbdl.SpatialTransform()
Xs2.r = np.array([0.,0.,0.])
Xs2.E = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

axis = np.asarray([
    [0., 0., 0., 1., 0., 0.],
    [0., 0., 0., 0., 1., 0.],
    [0., 0., 0., 0., 0., 1.],
    [0., 0., 1., 0., 0., 0.],
    [0., 1., 0., 0., 0., 0.],
    [1., 0., 0., 0., 0., 0.],
    ])

joint6Dof = rbdl.Joint.fromJointAxes (axis)

link1 = rbdl.Body.fromMassComInertia(m1,c1,J1)
link2 = rbdl.Body.fromMassComInertia(m2,c2,J2)

iLink1= model.AppendBody(rbdl.SpatialTransform(),joint6Dof,link1)
iLink2= model.AppendBody(rbdl.SpatialTransform(),joint6Dof,link2)

#point_coords = np.array ([0., 0., 1.])
q   = np.zeros (model.q_size)
qd  = np.zeros (model.qdot_size)
qdd = np.zeros (model.qdot_size)
tau = np.zeros (model.qdot_size)        

assert(iLink1==6)
assert(iLink2==12)

cs = rbdl.ConstraintSet()
cs.AddLoopConstraint(0,iLink1,Xp1,Xs1,rbdl.SpatialVector(0,[0,0,0,1,0,0]),False,0.1,"LoopGroundLink1",7)
cs.AddLoopConstraint(0,iLink1,Xp1,Xs1,rbdl.SpatialVector(0,[0,0,0,0,1,0]),False)
cs.AddLoopConstraint(0,iLink1,Xp1,Xs1,rbdl.SpatialVector(0,[0,0,0,0,0,1]),False)
cs.AddLoopConstraint(0,iLink1,Xp1,Xs1,rbdl.SpatialVector(0,[1,0,0,0,0,0]),False)
cs.AddLoopConstraint(0,iLink1,Xp1,Xs1,rbdl.SpatialVector(0,[0,1,0,0,0,0]),False)

cs.AddLoopConstraint( iLink1, iLink2, Xp2, Xs2, rbdl.SpatialVector(0,[0,0,0,1,0,0]),False,0.1,"LoopLink1Link2",11)
cs.AddLoopConstraint( iLink1, iLink2, Xp2, Xs2, rbdl.SpatialVector(0,[0,0,0,0,1,0]))
cs.AddLoopConstraint( iLink1, iLink2, Xp2, Xs2, rbdl.SpatialVector(0,[0,0,0,0,0,1]))
cs.AddLoopConstraint( iLink1, iLink2, Xp2, Xs2, rbdl.SpatialVector(0,[1,0,0,0,0,0]))
cs.AddLoopConstraint( iLink1, iLink2, Xp2, Xs2, rbdl.SpatialVector(0,[0,0,1,0,0,0]))

cs.Bind(model)
# self.qdot[5] = -1.6

contact_body_id = body_1
contact_point = np.array( [0., -1., 0.]);
contact_point = np.array( [0., -1., 0.])

constraint_set = rbdl.ConstraintSet()
constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([1., 0., 0.]), "ground_x");
constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([0., 1., 0.]), "ground_y");
constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([0., 0., 1.]), "ground_z");
constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([1., 0., 0.]), "ground_x")
constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([0., 1., 0.]), "ground_y")
constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([0., 0., 1.]), "ground_z")

constraint_set.Bind (model);
constraint_set.Bind (model)

# C code for the four bar linkage:
struct FourBarLinkage {

FourBarLinkage()
model()
cs()
q()
qd()
qdd()
tau()
l1(2.)
l2(2.)
m1(2.)
m2(2.)
idB1(0)
idB2(0)
idB3(0)
idB4(0)
idB5(0)
X_p(Xtrans(Vector3d(l2, 0., 0.)))
X_s(Xtrans(Vector3d(0., 0., 0.))) {

Body link1 = Body(m1, Vector3d(0.5 * l1, 0., 0.)
, Vector3d(0., 0., m1 * l1 * l1 / 3.));
Body link2 = Body(m2, Vector3d(0.5 * l2, 0., 0.)
, Vector3d(0., 0., m2 * l2 * l2 / 3.));
Vector3d vector3d_zero = Vector3d::Zero();
Body virtual_body(0., vector3d_zero, vector3d_zero);
Joint joint_rev_z(JointTypeRevoluteZ);
# AddBody(parent id, joint_frame, joint, body, body name)
# Parameters:
# parent_id:   id of the parent body
# joint_frame: the transformation from the parent frame to the origin of the joint frame (represents X_T in RBDA)
# joint:       specification for the joint that describes the connection
# body:        specification of the body itself
# body_name:   human readable name for the body (can be used to retrieve its id with GetBodyId())
idB1 = model.AddBody(0   , Xtrans(Vector3d(0., 0., 0.)),joint_rev_z, link1);
idB2 = model.AddBody(idB1, Xtrans(Vector3d(l1, 0., 0.)),joint_rev_z, link2);
idB3 = model.AddBody(0   , Xtrans(Vector3d(0., 0., 0.)),joint_rev_z, link1);
idB4 = model.AddBody(idB3, Xtrans(Vector3d(l1, 0., 0.)),joint_rev_z, link2);
idB5 = model.AddBody(idB4, Xtrans(Vector3d(l2, 0., 0.)),joint_rev_z, virtual_body);
# AddLoopConstraint   (id_predecessor, id_successor,SpatialTransform &  X_predecessor,SpatialTransform &  X_successor,SpatialVector&axis,enable_stabilization = false,stabilization_param = 0.1,constraint_name = NULL)   
cs.AddLoopConstraint(idB2, idB5, X_p, X_s,SpatialVector(0,0,0,1,0,0), true, 0.1);
cs.AddLoopConstraint(idB2, idB5, X_p, X_s,SpatialVector(0,0,0,0,1,0), true, 0.1);
cs.AddLoopConstraint(idB2, idB5, X_p, X_s,SpatialVector(0,0,1,0,0,0), true, 0.1);

cs.Bind(model);

q = VectorNd::Zero(model.dof_count);
qd = VectorNd::Zero(model.dof_count);
qdd = VectorNd::Zero(model.dof_count);
tau = VectorNd::Zero(model.dof_count);

  }

  Model model;
  ConstraintSet cs;

  VectorNd q;
  VectorNd qd;
  VectorNd qdd;
  VectorNd tau;

  double l1;
  double l2;
  double m1;
  double m2;

  unsigned int idB1;
  unsigned int idB2;
  unsigned int idB3;
  unsigned int idB4;
  unsigned int idB5;

  SpatialTransform X_p;
  SpatialTransform X_s;

};
