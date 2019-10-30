#!/usr/bin/env python
import numpy as np
from sympy import *
from ambf_client import Client
import time

from MCG import *

_client = Client()
_client.connect()

print(_client.get_obj_names())

mtm_handle = _client.get_obj_handle('mtm/TopPanel')
mtmPS_handle = _client.get_obj_handle('mtm/OutPitchShoulder')
time.sleep(1)

q_old = mtm_handle.get_all_joint_pos()
q_old = np.array(q_old[:7])
q = q_old

# _client.clean_up()

t = time.time()

for i in range(1000):
    dt = time.time()-t
    t = time.time()
    print('freq = ', 1/dt)
    

    q = np.array(mtm_handle.get_all_joint_pos())
    # q = [0]*9
    # for i in range(0,9):
    #     q[i] = mtm_handle.get_joint_pos(i)

    if len(q)==0:
        print 'q is empty'
        break
    
    q_trunc = np.array(q[:7])
    qdot = (q_trunc - q_old)/dt
    
    M = get_M(q_trunc)
    C = get_C(q_trunc, qdot)
    G = get_G(q_trunc)
    print(G.shape)

    Tau = G[1]
    mtm_handle.set_joint_effort(1, Tau)

    q_old = q_trunc
    time.sleep(0.01)
_client.clean_up()
