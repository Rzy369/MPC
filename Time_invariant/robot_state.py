import numpy as np

class robot_state():
    def __init__(self, z = 0, gamma = 0):
        self.z = z
        self.gamma = gamma

    def state_update(self, v, w):
        dt = 0.1
        self.z += v * dt
        self.gamma += w * dt