import numpy as np

class needle_model_for_prediction():
    def __init__(self):
        pass
    def sol_Matrix(self, alpha_m, beta_m, gamma_m, k):
        '''
        According to the paper, the kinematics model is 
        q_(m+1) = q_m + dq_m * dt,
        when selecting u = [a, theta],
        hereby the theta is the angle rotated about the z-axis.
        the model can be transfered to x = Ax + Bu
        '''
        dt = 0.1        #[s]                    # Time step

        A = np.eye(7)
        A[6, 0] = np.sin(beta_m) * dt
        A[6, 1] = -np.cos(beta_m) * np.sin(alpha_m) * dt
        A[6, 2] = np.cos(alpha_m) * np.cos(beta_m) * dt
        A[6, 3] = k * np.cos(gamma_m) / np.cos(beta_m) * dt
        A[6, 4] = k * np.sin(gamma_m) * dt
        A[6, 5] = -k * np.cos(gamma_m) * np.tan(beta_m) * dt

        B = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 1],
                      [dt, 0]])

        return A, B