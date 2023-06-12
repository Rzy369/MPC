import numpy as np

class needle_model():
    def __init__(self):
        pass
    def sol_Matrix(self):
        '''
        According to the paper, the kinematics model is 
        q_(m+1) = q_m + dq_m * dt,
        when selecting u = [v, w],
        and linearized around initial point,
        the model can be transfered to x = Ax + Bu
        '''
        dt = 0.1        #[s]                    # Time step

        A = np.eye(2)

        B = dt * np.eye(2)

        return A, B

