import numpy as np
from scipy.linalg import solve_discrete_are

def lqr_controller(x_ref, initial_state, model, k, path):
    v0 = 0                          #[m/s]          # Initial speed
    N = 100                 #[]             # Predict Horizon
    # reference path
    path_ref = np.vstack([path.z_ref, path.v_ref])
    # state and input cost matrix
    Q = 1*np.diag([10, 0.1])
    R = 0.1*np.diag([1, 1])

    # Initial state，input and gain variables
    P = [None] * (N + 1)
    x = [None] * (N + 1)
    K = [None] * N
    u = [None] * N

    # adding x = x_initial as the constraint
    x_initial = np.array([initial_state.z, initial_state.v])
    
    # system dynamics matrix A, B
    alpha_initial = initial_state.alpha
    beta_initial = initial_state.beta
    gamma_initial = initial_state.gamma
    A, B = model.sol_linearized_Matrix()
    
    # terminal cost matrix Qf
    # P[N]= solve_discrete_are(A, B, Q, R)  # terminal cost matrix
    P[N] = np.array([[10.0990195127249,0.00999054509851694],[0.0099905450985169, 1.05675505807871]])
    # Controller loop
    # iterative backward discrete algebraic Riccati equation (IBDA)
    for i in range(N, 0, -1):
        P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)
    
    costs = 0
    x[0] = x_initial
    for i in range(N):
        # calculate the optimal feedback gain K
        K[i] = np.linalg.pinv(R + B.T @ P[i+1] @ B) @ (B.T @ P[i+1] @ A)
        u[i] = -K[i] @ (x[i] - path_ref[:, i])
        # update system states
        x[i+1] = A @ x[i] + B @ u[i]   
        # adding the costs
        costs += cvxpy.quad_form(x[i] - path_ref[:, i], Q)    # state cost
        costs += cvxpy.quad_form(u[i], R)                  # input cost
    costs += cvxpy.quad_form(x[N] - path_ref[:, N], P[N])     # terminal cost

    return x, u
