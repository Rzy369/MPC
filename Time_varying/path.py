import robot_state
import random
import numpy as np

class path:
    '''
    Using lists to record the ref path
    '''
    def __init__(self, x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, v_ref, w_ref):
        self.x_ref = x_ref
        self.y_ref = y_ref
        self.z_ref = z_ref
        self.alpha_ref = alpha_ref
        self.beta_ref = beta_ref
        self.gamma_ref = gamma_ref
        self.v_ref = v_ref
        self.w_ref = w_ref

def setting_path(v, w, kappa):
    rob = rob = robot_state.robot_state(0,0,0,0,0,0,v,w)

    step = 100

    x = []
    y = []
    z = []
    alpha = []
    beta = []
    gamma = []
    v_ref = []
    w_ref = []

    x.append(rob.x)
    y.append(rob.y)
    z.append(rob.z)
    alpha.append(rob.alpha)
    beta.append(rob.beta)
    gamma.append(rob.gamma)
    v_ref.append(rob.v)
    w_ref.append(rob.w)

    for i in range(50):
        rob.state_update(v,w,kappa)
        x.append(rob.x)
        y.append(rob.y)
        z.append(rob.z)
        alpha.append(rob.alpha)
        beta.append(rob.beta)
        gamma.append(rob.gamma)
        v_ref.append(rob.v)
        w_ref.append(rob.w)

    for i in range(10):
        rob.state_update(0.009-0.001*i,w,kappa)
        x.append(rob.x)
        y.append(rob.y)
        z.append(rob.z)
        alpha.append(rob.alpha)
        beta.append(rob.beta)
        gamma.append(rob.gamma)
        v_ref.append(rob.v)
        w_ref.append(rob.w)

    ref_path = path(x, y, z, alpha, beta, gamma, v_ref, w_ref)

    return ref_path