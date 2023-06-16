import numpy as np

class robot_state():
    def __init__(self, x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0, w = 0):
        self.x = x
        self.y = y
        self.z = z
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.v = v
        self.w = w

    def state_update(self, v, w, kappa):
        '''
        Updating the state of robot,
        a is the acceleration of the velocity v,
        theta is the angel rotated around the z axis.
        '''
        dt = 0.1
        self.x += v * np.sin(self.beta) * dt * 1000
        self.y += v * (-np.cos(self.beta) * np.sin(self.alpha)) * dt * 100
        self.z += v * (np.cos(self.alpha) * np.cos(self.beta)) * dt * 100
        self.alpha += v * (kappa * np.cos(self.gamma) / np.cos(self.beta)) * dt
        self.beta += v * kappa * np.sin(self.gamma) * dt
        self.gamma += v * (-kappa * np.cos(self.gamma) * np.tan(self.beta)) * dt + w * dt
        self.v = v
        self.w = w