import robot_state
import random
import numpy as np

class ref_path:
    '''
    Using lists to record the ref path
    '''
    def __init__(self, x_ref, y_ref, z_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.z_ref = z_ref

def setting_easy_ref_path(a, theta):
    rob = robot_state.robot_state()
    
    step = 100                          # step between stages
    k = 1/150

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

    path = ref_path(x, y, z)

    return path

def generate_rand_ref_path():
    rob = robot_state.robot_state()

    step = 100                          # step between stages
    k = 1/150
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
    
    path = ref_path(x, y, z)

    return path
