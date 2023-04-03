import numpy as np

class robot_state():
    def __init__(self, x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0):
        self.x = x
        self.y = y
        self.z = z
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.v = v

    def state_update(self, a, theta, k):
        '''
        Updating the state of robot,
        a is the acceleration of the velocity v,
        theta is the angel rotated around the z axis.
        '''
        dt = 0.1
        self.x += self.v * np.sin(self.beta) * dt
        self.y += self.v * (-np.cos(self.beta) * np.sin(self.alpha)) * dt
        self.z += self.v * (np.cos(self.alpha) * np.cos(self.beta)) * dt
        self.alpha += self.v * (k * np.cos(self.gamma) / np.cos(self.beta)) * dt
        self.beta += self.v * k * np.sin(self.gamma) * dt
        self.gamma += self.v * (-k * np.cos(self.gamma) * np.tan(self.beta)) * dt + theta
        self.v += a * dt