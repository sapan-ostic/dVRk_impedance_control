## The following code snippet is Written by Ajith Kumar

import numpy as np
import scipy as sp
import math
from numpy.linalg import inv
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Global variables 
I1 = 10
I2= 10 
I3 =10
m1 = 5
m2 = 5
m3 = 5
r1 = 0.5
r2 = 0.5
r3 = 0.5
l1 = 1
l2 = 1
l3 = 1
g=9.8


def set_point_CTC(x,t):
    

    # Desired theta
    theta_d = np.array([2,1,3])
    dtheta_d = np.array([2,4,5])
    ddtheta_d = np.array([0,0,0])
    
    theta = x[0:3]
    dtheta = x[3:6]
    
    # M,C,G matrix parameters
    Mmat = np.array([[ I1 + I2 + I3 + r1**2*m2 + r1**2*m3 + r2**2*m3 + l1**2*m1 + l2**2*m2 + l3**2*m3 + 2*r1*l3*m3*math.cos(x[1] + x[2]) + 2*r1*r2*m3*math.cos(x[1]) + 2*r1*l2*m2*math.cos(x[1]) + 2*r2*l3*m3*math.cos(x[2]), m3*r2**2 + 2*m3*math.cos(x[2])*r2*l3 + r1*m3*math.cos(x[1])*r2 + m2*l2**2 + r1*m2*math.cos(x[1])*l2 + m3*l3**2 + r1*m3*math.cos(x[1] + x[2])*l3 + I2 + I3, I3 + l3**2*m3 + r1*l3*m3*math.cos(x[1] + x[2]) + r2*l3*m3*math.cos(x[2])]
        ,[m3*r2**2 + 2*m3*math.cos(x[2])*r2*l3 + r1*m3*math.cos(x[1])*r2 + m2*l2**2 + r1*m2*math.cos(x[1])*l2 + m3*l3**2 + r1*m3*math.cos(x[1] + x[2])*l3 + I2 + I3,m3*r2**2 + 2*m3*math.cos(x[2])*r2*l3 + m2*l2**2 + m3*l3**2 + I2 + I3,m3*l3**2 + r2*m3*math.cos(x[2])*l3 + I3]
        ,[I3 + l3**2*m3 + r1*l3*m3*math.cos(x[1] + x[2]) + r2*l3*m3*math.cos(x[2]), m3*l3**2 + r2*m3*math.cos(x[2])*l3 + I3, m3*l3**2 + I3]])

    Cmat = np.array([[ - r1*r2*m3*x[4]*math.sin(x[1]) - r1*l2*m2*x[4]*math.sin(x[1]) - r2*l3*m3*x[5]*math.sin(x[2]) - r1*l3*m3*x[4]*math.sin(x[1] + x[2]) - r1*l3*m3*x[5]*math.sin(x[1] + x[2]), - r1*l3*m3*math.sin(x[1] + x[2]) - r1*r2*m3*math.sin(x[1]) - r1*l2*m2*math.sin(x[1]) - r1*r2*m3*x[4]*math.sin(x[1]) - r1*l2*m2*x[4]*math.sin(x[1]) - r2*l3*m3*x[5]*math.sin(x[2]) - r1*l3*m3*x[4]*math.sin(x[1] + x[2]) - r1*l3*m3*x[5]*math.sin(x[1] + x[2]),-l3*m3*(r2*math.sin(x[2]) + r1*math.sin(x[1] + x[2]))*(x[4] + x[5] + 1)]
       ,[ r1*r2*m3*x[3]*math.sin(x[1]) + r1*l2*m2*x[3]*math.sin(x[1]) - r2*l3*m3*x[5]*math.sin(x[2]) + r1*l3*m3*x[3]*math.sin(x[1] + x[2]),(r1*r2*m3*x[3]*math.sin(x[1]))/2 - (r1*r2*m3*math.sin(x[1]))/2 - (r1*l2*m2*math.sin(x[1]))/2 - (r1*l3*m3*math.sin(x[1] + x[2]))/2 + (r1*l2*m2*x[3]*math.sin(x[1]))/2 - r2*l3*m3*x[5]*math.sin(x[2]) + (r1*l3*m3*x[3]*math.sin(x[1] + x[2]))/2, (r1*l3*m3*x[3]*math.sin(x[1] + x[2]))/2 - r2*l3*m3*math.sin(x[2]) - r2*l3*m3*x[4]*math.sin(x[2]) - r2*l3*m3*x[5]*math.sin(x[2]) - (r1*l3*m3*math.sin(x[1] + x[2]))/2 ] 
       ,[r2*l3*m3*x[3]*math.sin(x[2]) + r2*l3*m3*x[4]*math.sin(x[2]) + r1*l3*m3*x[3]*math.sin(x[1] + x[2]), r2*l3*m3*x[3]*math.sin(x[2]) - (r1*l3*m3*math.sin(x[1] + x[2]))/2 + r2*l3*m3*x[4]*math.sin(x[2]) + (r1*l3*m3*x[3]*math.sin(x[1] + x[2]))/2,(l3*m3*(x[3] - 1)*(r2*math.sin(x[2])) + r1*math.sin(x[1] + x[2]))/2]])

    Gmat = np.array([[r1*m2*math.cos(x[0]) + r1*m3*math.cos(x[0]) + l1*m1*math.cos(x[0]) + l3*m3*math.cos(x[0] + x[1] + x[2]) + r2*m3*math.cos(x[0] + x[1]) + l2*m2*math.cos(x[0] + x[1])]
       ,[l3*m3*math.cos(x[0] + x[1] + x[2]) + r2*m3*math.cos(x[0] + x[1]) + l2*m2*math.cos(x[0] + x[1])]
       ,[l3*m3*math.cos(x[0] + x[1] + x[2])]])

    
    invM = inv(Mmat)
    invMG = np.matmul(invM,Gmat)
    invMC = np.matmul(invM,Cmat)

    e = theta_d - theta
    de = dtheta_d -dtheta
    e = e.reshape(3,1)
    de = de.reshape(3,1)

    Kp = 100*np.eye(3)
    Kd = 15*np.eye(3)
    
    tau = np.matmul(Mmat,(np.matmul(Kp,e) + np.matmul(Kd,de))) + np.matmul(Cmat,dtheta.reshape(3,1)) \
          + np.matmul(Mmat,ddtheta_d.reshape(3,1)) + Gmat
    
    # beta = 0.1
    # tau = (1/beta)*(np.matmul(Kp,p_e) + np.matmul(Kd,v_e)) 
    
    j = -np.matmul(invMC,dtheta.reshape(3,1)) + np.matmul(invM,tau) - np.matmul(invM,Gmat)

    dx = np.zeros((6,))
    dx[0] = x[3]
    dx[1] = x[4]    
    dx[2] = x[5]
    dx[3] = j[0]
    dx[4] = j[1]
    dx[5] = j[2]
    return dx


if __name__ == '__main__':

    # Initial point
    x0 = np.array([-0.5,0.8,0.9,0.1,0.1,0.1])
    
    # Final point 
    Xf = np.array([0.2,0.2,0.2,0,0,0])
    
    # Final Time
    tf = 15
    t = np.linspace(0,tf,1501)

    X = np.zeros((4,1))
    # set_point_CTC(x0,t)
    X = odeint(set_point_CTC,x0,t)

    plt.plot(t,X[:,0],'r',label='theta1')
    plt.plot(t,X[:,1],'b',label='theta2')
    plt.plot(t,X[:,2],'g',label='theta3')
    plt.plot(t,X[:,2],'g',label='dtheta1')
    plt.plot(t,X[:,3],'c',label='dtheta2')
    plt.plot(t,X[:,5],'y',label='dtheta3')
    plt.xlabel('t')
    plt.yticks()
    plt.legend()
    plt.show()
