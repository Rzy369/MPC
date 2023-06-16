import robot_state
import random
import numpy as np

class path:
    '''
    Using lists to record the ref path
    '''
    def __init__(self, z_ref, gamma_ref):
        self.z_ref = z_ref
        self.gamma_ref = gamma_ref

def setting_path(v,w):
    rob = rob = robot_state.robot_state()

    step = 100

    z = []
    gamma = []

    for i in range(step-10):
        rob.state_update(v,w)
        z.append(rob.z)
        gamma.append(rob.gamma)

    for i in range(10):
        rob.state_update(0.09 - 0.01*i,w)
        z.append(rob.z)
        gamma.append(rob.gamma)

    ref_path = path(z, gamma)

    return ref_path