import numpy as np

class needle_model():
    def __init__(self):
        pass

    def sol_Matrix(self, alpha_0, beta_0, gamma_0, v_0, w_0, kappa):
        '''
        According to the paper, the kinematics model is 
        q_(m+1) = q_m + dq_m * dt,
        when selecting u = [v, w],
        and linearized around trajectory,
        the model can be transfered to x = A(x - x_0) + B(u - u_0) + x_0
        '''
        dt = 0.1        #[s]                    # Time step

        A = np.eye(6)
        A[0, 3] = np.cos(beta_0) * v_0 * dt
        A[1, 3] = -np.cos(alpha_0) * np.cos(beta_0) * v_0 * dt
        A[1, 4] = np.sin(alpha_0) * np.sin(beta_0) * v_0 * dt
        A[2, 3] = -np.sin(alpha_0) * np.cos(beta_0) * v_0 * dt
        A[2, 4] = -np.cos(alpha_0) * np.sin(beta_0) * v_0 * dt
        A[3, 4] = kappa * np.cos(gamma_0) * np.tan(beta_0) / np.cos(beta_0) * v_0 * dt
        A[3, 5] = -kappa * np.sin(gamma_0) / np.cos(beta_0) * v_0 * dt
        A[4, 5] = kappa * np.cos(gamma_0) * v_0 * dt
        A[5, 4] = -kappa * np.cos(gamma_0) / ((np.cos(beta_0)) ** 2) * v_0 * dt
        A[5, 5] = kappa * np.sin(gamma_0) * np.tan(beta_0) * v_0 * dt

        B = np.array([[np.sin(beta_0), 0],
                      [-np.cos(beta_0) * np.sin(alpha_0), 0],
                      [np.cos(alpha_0)*np.cos(beta_0), 0],
                      [kappa * np.cos(gamma_0) / np.cos(beta_0), 0],
                      [kappa * np.sin(gamma_0), 0],
                      [-kappa * np.cos(gamma_0) * np.tan(beta_0), w_0*dt]])
        return A,B
