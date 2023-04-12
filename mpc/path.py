import robot_state
import random
import numpy as np

class path:
    '''
    Using lists to record the ref path
    '''
    # def __init__(self, x_ref, y_ref, z_ref):
    #     self.x_ref = x_ref
    #     self.y_ref = y_ref
    #     self.z_ref = z_ref
    def __init__(self, x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, v_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.z_ref = z_ref
        self.alpha_ref = alpha_ref
        self.beta_ref = beta_ref
        self.gamma_ref = gamma_ref
        self.v_ref = v_ref

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
    a_max = 1
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

def easy_path(a, theta, k):
    rob = robot_state.robot_state()
    
    step = 100                          # step between stages

    x = []
    y = []
    z = []
    alpha = []
    beta = []
    gamma = []
    v = []
    x.append(rob.x)
    y.append(rob.y)
    z.append(rob.z)
    alpha.append(rob.alpha)
    beta.append(rob.beta)
    gamma.append(rob.gamma)
    v.append(rob.v)
    for i in range(step):
        rob.state_update(a, theta, k)
        x.append(rob.x)
        y.append(rob.y)
        z.append(rob.z)
        alpha.append(rob.alpha)
        beta.append(rob.beta)
        gamma.append(rob.gamma)
        v.append(rob.v)

    ref_path = path(x, y, z, alpha, beta, gamma, v)

    return ref_path