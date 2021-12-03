#!/usr/bin/env python
from __future__ import print_function
from multiprocessing import Process,Pipe
import numpy as np
import random

import sys
print(sys.path)
sys.path.append('/home/')
sys.path.append('/home/mcw/01_Research/01_Projects/koopman/koopman_policy/envs')
sys.path.append('/home/mcw/01_Research/01_Projects/koopman/koopman_policy')
sys.path.append('/home/mcw/01_Research/01_Projects/koopman/')
import koopman_policy.envs
#import koopman_policy.envs
#from envs.pivoting import PivotingEnv
from garage.experiment import Snapshotter
import time
import zmq
import json
from datetime import datetime



def get_koopman_actions_from_extra(obs_msg):

    a, agent_info = policy.get_action(obs_msg)
    deterministic=True #is this true?

    if deterministic and 'mean' in agent_info:
        a = agent_info['mean']

    lb, ub = env.action_space.low, env.action_space.high
    if np.all(lb != -np.inf) and np.all(ub != -np.inf):
        scaled_action = lb + (a + env._expected_action_scale) * (
            0.5 * (ub - lb) / env._expected_action_scale)
        scaled_action = np.clip(scaled_action, lb, ub) 

    
    #dummy action 
    acc=scaled_action[0] #0 + random.uniform(-1.1, 1.1)*random.randint(0,1)
    acc*=-1.
    finger_distance=scaled_action[1]#0.01+ random.uniform(-0.01, 0.01)*random.randint(0,1)
    
    return np.array([acc, finger_distance])


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:50165")
import re

pivot_polecy_loc="/home/mcw/01_Research/01_Projects/koopman/koopman_policy/pivot_models/koopmanlqr_ppo_pivoting_tests_102"
snapshotter = Snapshotter()
print(pivot_polecy_loc)
data = snapshotter.load(pivot_polecy_loc)
policy = data['algo'].policy
env = data['env']
policy.reset()#necesary?
policy._kpm_ctrl.training = False

try:
    while True:
        #  Wait for next request from client
        message = socket.recv()
        obs=message.decode("utf-8")
        obs = obs.replace('[','')
        obs = obs.replace(']','')
        obs = np.fromstring(obs, dtype=float, sep=" ")
        action=get_koopman_actions_from_extra(obs) 
        #  Send reply back to client
        socket.send(bytes(np.array2string(action), encoding='utf-8'))

except KeyboardInterrupt:
    print("Shutting down koopman")




