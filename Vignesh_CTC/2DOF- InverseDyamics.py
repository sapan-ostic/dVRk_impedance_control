import numpy as np
import scipy as sp
import math
from numpy.linalg import inv
import matplotlib.pyplot as plt
from scipy.integrate import odeint

I1, I2, I3 = 10, 10, 10
m1, m2 , m3 = 5, 5, 5
l1, l2, l3 = 1, 1, 1
r1, r2, r3 = 0.5, 0.5, 0.5
g = 9.8
torque = 0

def trajgenerator(th1_i, dth1_i, th1_f, dth1_f, tf):
    M = np.array([[1, 0, 0, 0],[0, 1, 0, 0], [1, tf, tf**2, tf**3], [0, 1, 2*tf, 3*tf**2]])
    # print "M val ", M
    b = np.array([[th1_i], [dth1_i], [th1_f], [dth1_f]])
    # print "b val ", b.shape
    # print "inv M ", inv(M).shape
    a = np.dot(inv(M), b)
    print " a val is ", a
    return a

def inverse_dynamics(x, t, a1_link, a2_link):

    kp1, kp2, kd1, kd2 = 200, 300, 150, 300
    kp = np.array([[kp1, 0], [0, kp2]])
    kd = np.array([[kd1, 0], [0, kd2]])

    a = I1 + I2 + m1 * r1 ** 2 + m2 * (l1 ** 2 + r2 ** 2)
    b = m2 * l1 * r2
    d = I2 + m2 * r2 ** 2

    Mmat = np.array([[a + 2 * b * math.cos(x[1]), d + b * math.cos(x[1])], [d + b * math.cos(x[1]), d]])
    Cmat = np.array([[-b * math.sin(x[1]) * x[3], -b * math.sin(x[1]) * x[2] + x[1]], [b * math.sin(x[1]) * x[2], 0]])
    Gmat = np.array([[m1 * g * r1 * math.cos(x[0]) + m2 * g * (l1 * math.cos(x[0]) + r2 * math.cos(x[0] + x[1]))], [m2 * g * r2 * math.cos(x[0] + x[1])]])
    invM = inv(Mmat)

    vec_t = np.array([[1, t, t**2, t**3]])
    print "vect is ", vec_t
    dvec_t = np.array([[0, 1, 2*t, 3*t**2]])
    print " org dvect ", dvec_t.shape
    print " reshape d vect ", np.reshape(dvec_t, (1, 4))
    ddvec_t = np.array([[0, 0, 2, 6* t]])
    print "d vect shape ", dvec_t.shape
    link_1 = np.array([[a1_link[0]], [a1_link[1]], [a1_link[2]], [a1_link[3]]])
    link_2 = np.array([[a2_link[0]], [a2_link[1]], [a2_link[2]], [a2_link[3]]])
    print "link shape ", link_1
    print " reshape link ",  np.reshape(link_1, (4, 1))

    theta_d = np.array([[np.dot(vec_t, np.reshape(link_1, (4, 1)))], [np.dot(vec_t, np.reshape(link_2, (4, 1)))]])

    dtheta_d = np.array([[np.dot(dvec_t, np.reshape(link_1, (4, 1)))], [np.dot(dvec_t, np.reshape(link_1, (4, 1)))]])
    ddtheta_d = np.array([[np.dot(ddvec_t, np.reshape(link_1, (4, 1)))], [np.dot(ddvec_t, np.reshape(link_2, (4, 1)))]])

    theta = np.array([[x[0], x[1]]])
    dtheta = np.array([[x[2]], [x[3]]])

    e = np.array([[x[0] - theta_d[0]], [x[1] - theta_d[1]]])
    de = np.array([[x[2] - dtheta_d[0]], [x[3] - dtheta_d[1]]])
    Aq_d = ddtheta_d
    Aq = np.reshape(Aq_d, (2, 1)) - np.dot(kp, np.reshape(e, (2, 1))) - np.dot(kd, np.reshape(de, (2, 1)))

    Umat = np.dot(Mmat, Aq) + np.dot(Cmat, np.reshape(dtheta_d, (2, 1))) + Gmat

    dim = 4  # number of entries
    shp = (dim, 1)  # shape tuple
    dx = np.zeros(shp)
    # print "dx is ", dx

    dx_temp = np.dot(inv(Mmat), (Umat - np.dot(Cmat, dtheta) - Gmat))
    # print "dx temp is ", dx_temp[0][0]

    dx = [x[2], x[3], dx_temp[0][0], dx_temp[1][0]]
    print " dx is ", dx
    return dx

if __name__ == '__main__':

    x0 = np.array([50, 60, 0.5, 0.3])
    x1 = np.array([30, 50, 0.3, 0.4])
    xf = np.array([2, 1, 2, 4])
    tf = 15
    th1_i = x0[0]
    th2_i = x0[1]
    dth1_i = x0[2]
    dth2_i = x0[3]

    # Final state
    th1_f = xf[0]
    th2_f = xf[1]
    dth1_f = xf[2]
    dth2_f = xf[3]

    link1 = trajgenerator(th1_i, dth1_i, th1_f, dth1_f,tf)
    link2 = trajgenerator(th2_i, dth2_i, th2_f, dth2_f,tf)

    t = np.linspace(0, 15, num=1501)
    y0 = x0
    sol = odeint(inverse_dynamics, y0, t, args=(link1, link2))
    sol2 = odeint(inverse_dynamics, x1, t, args=(link1, link2))
    plt.plot(t, sol[:, 0], 'b', t, sol2[:, 0], 'y', label='Theta_1 under inverse dynamic control')
    plt.plot(t, sol[:, 1], 'g', t, sol2[:, 1], 'r', label='Theta_2 under inverse dynamic control')
    plt.legend(loc='best')
    plt.xlabel('t')
    plt.grid()
    plt.show()