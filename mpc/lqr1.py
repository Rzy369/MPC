import numpy as np
import cvxpy 
from scipy.linalg import solve_discrete_are

def lqr_controller(x_ref, initial_state, model, k, path):
    v0 = 0                          #[m/s]          # Initial speed
    N = 6                         #[]             # Predict Horizon
    # a_max = 3                       #[m/s^2]        # Max acceleration
    # a_min = -1                      #[m/s^2]        # Min acceleration
    # theta_max = np.deg2rad(60)      #[]             # Max theta per time step
    # theta_min = -np.deg2rad(60)     #[]             # Min theta per time step

    Q = np.diag([10, 10, 10, 0.1, 0.1, 0.1, 0.1])          # weighting matrix
    R = np.diag([1, 1])                             # weighting matrix 

    # Initial state，input and gain variables
    P = [None] * (N + 1)
    # x = np.zeros(7, 1, (N + 1))
    x = [None] * (N + 1)
    K = [None] * N
    u = [None] * N

    # adding x = x_initial as the constraint
    x_initial = np.array([initial_state.x, initial_state.y, initial_state.z,
                            initial_state.alpha, initial_state.beta, initial_state.gamma,
                            initial_state.v])
    # constraints += [x[:, 0] == x_initial]
    
    # system dynamics matrix A, B
    # alpha_initial = initial_state.alpha
    # beta_initial = initial_state.beta
    # gamma_initial = initial_state.gamma
    alpha_final = x_ref[3, N]
    beta_final = x_ref[4, N]
    gamma_final = x_ref[5, N]
    A, B = model.sol_Matrix(alpha_final, beta_final, gamma_final, k)
    
    # terminal cost matrix Qf
    P[N]= solve_discrete_are(A, B, Q, R)  # terminal cost matrix
    
    # Constraints
    # constraints = []
    # constraints.append(x[0] == x0)
    # for i in range(N):
    #     constraints += [x[:, i+1] == A @ x[:, i] + B @ u[:, i]]
    #     constraints += [u[0, i] <= a_max]                      
    #     constraints += [u[0, i] >= a_min]
    #     constraints += [u[1, i] <= theta_max]
    #     constraints += [u[1, i] >= theta_min]

    # Cost function
    # cost = 0
    # for i in range(N):
    #     costs += cvxpy.quad_form(x[:, i] - x_ref[:, i], Q)    # state cost
    #     costs += cvxpy.quad_form(u[:, i], R)                  # input cost
    # cost += cvxpy.quad_form(x[:, N], P[N+1])

    # Optimization problem
    # prob = cp.Problem(cp.Minimize(cost), constraints)

    # Controller loop
    # iterative backward discrete algebraic Riccati equation (IBDA)
    for i in range(N, 0, -1):
        P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)
    
    costs = 0
    x[0] = x_initial
    for i in range(N):
        # calculate the optimal feedback gain K
        K[i] = np.linalg.pinv(R + B.T @ P[i+1] @ B) @ (B.T @ P[i+1] @ A)
        u[i] = -K[i] @ x[i]
        x[i+1] = A @ x[i] + B @ u[i]
        # adding the costs
        costs += cvxpy.quad_form(x[i] - x_ref[:, i], Q)    # state cost
        costs += cvxpy.quad_form(u[i], R)                  # input cost
    costs += cvxpy.quad_form(x[N] - x_ref[:, N], P[N])     # terminal cost




    # x_lqr = np.zeros((N + 1, 7))
    # u_lqr = np.zeros((N, 2))
    # x_lqr[0] = path.x_ref[0]
    # for i in range(N):
    #     x0.value = x_lqr[i]
    #     u0.value = u_lqr[i-1] if i > 0 else np.zeros(2)

    #     # Solve ARE for terminal cost
    #     P = Qf
    #     for j in range(N):
    #         P_new = A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
    #         if np.allclose(P_new, P):
    #             break
    #         P = P_new
    #     K[i] = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A

    #     # Solve LQR optimization problem
    #     prob.solve(solver=cp.OSQP)

    #     # Extract optimal input
    #     u_lqr[i] = u.value[0]

    #     # Update state
    #     x_lqr[i+1] = A @ x_lqr[i] + B @ u_lqr[i]

    return x, u
