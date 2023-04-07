import robot_state
import random
import numpy as np

class path:
    '''
    Using lists to record the ref path
    '''
    def __init__(self, x_ref, y_ref, z_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.z_ref = z_ref

def setting_easy_path(a, theta, k):
    rob = robot_state.robot_state()
    
    step = 100                          # step between stages

    x = []
    y = []
    z = []
    x.append(rob.x)
    y.append(rob.y)
    z.append(rob.z)

    for i in range(step):
        rob.state_update(a, theta, k)
        x.append(rob.x)
        y.append(rob.y)
        z.append(rob.z)

    ref_path = path(x, y, z)

    return ref_path

def generate_rand_path(k):
    rob = robot_state.robot_state()

    step = 100                          # step between stages
    a_max = 3
    a_min = -1
    theta_max = np.radians(60)
    theta_min = -np.radians(60)

    x = []
    y = []
    z = []
    x.append(rob.x)
    y.append(rob.y)
    z.append(rob.z)

    for i in range(step):
        theta = random.uniform(theta_min, theta_max)
        a = random.uniform(a_min, a_max)
        rob.state_update(a, theta, k)
        x.append(rob.x)
        y.append(rob.y)
        z.append(rob.z)
    
    ref_path = path(x, y, z)

    return ref_path

