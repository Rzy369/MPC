import numpy as np
from scipy.linalg import solve_discrete_are
import cvxpy

# def solve_DARE(A, B, Q, R):
#     # Solve the DARE
#     P = solve_discrete_are(A, B, Q, R)

#     # Print the solution
#     print("The solution to the DARE is:")
#     print(P)
#     return P

def lqr_controller(x_ref, initial_state, model, k, path):
    v0 = 0                          #[m/s]          # Initial speed
    N = 15                           #[]             # Predict Horizon
    a_max = 3                       #[m/s^2]        # Max acceleration
    a_min = -1                      #[m/s^2]        # Min acceleration
    theta_max = np.deg2rad(60)      #[]             # Max theta per time step
    theta_min = -np.deg2rad(60)     #[]             # Min theta per time step

    Q = np.diag([10, 10, 1, 0.1, 0.1, 0.1, 0.1])          # weighting matrix
    R = np.diag([1, 1])                             # weighting matrix

    # initializing the input and state
    x = cvxpy.Variable((7, N+1))
    u = cvxpy.Variable((2, N+1))
    
    # initializing the cost function and constraints
    costs = 0
    constraints = []
    
    # adding x = x_initial as the constraint
    x_initial = np.array([initial_state.x, initial_state.y, initial_state.z,
                            initial_state.alpha, initial_state.beta, initial_state.gamma,
                            initial_state.v])
    constraints += [x[:, 0] == x_initial]

    alpha_initial = initial_state.alpha
    beta_initial = initial_state.beta
    gamma_initial = initial_state.gamma
    A, B = model.sol_Matrix(alpha_initial, beta_initial, gamma_initial, k)
    
    # solve DARE function
    P = [None] * (N + 1)
    # X = [None] * (N + 1)
    K = [None] * N
    # U = [None] * N
        
    P[N] = solve_discrete_are(A, B, Q, R)
    
    # iterative backward discrete algebraic Riccati equation (IBDA)
    for i in range(N, 0, -1):
        P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)
        
    for i in range(N):
        # calculate the optimal feedback gain K
        K[i] = np.linalg.pinv(R + B.T @ P[i+1] @ B) @ (B.T @ P[i+1] @ A)
        u[i] = -K[i] @ x[:, i]
        # adding the constriants
        constraints += [x[:, i+1] == A @ x[:, i] + B @ u[:, i]]
        constraints += [u[0, i] <= a_max]                      
        constraints += [u[0, i] >= a_min]
        constraints += [u[1, i] <= theta_max]
        constraints += [u[1, i] >= theta_min]

        # adding the costs
        costs += cvxpy.quad_form(x[:, i] - x_ref[:, i], Q)    # state cost
        costs += cvxpy.quad_form(u[:, i], R)                  # input cost

    # costs += cvxpy.quad_form(x[:, N] - x_ref[:, N], Q)
    costs += cvxpy.quad_form(x[:, N] - x_ref[:, N], P[N])
    # solving
    problem = cvxpy.Problem(cvxpy.Minimize(costs), constraints)
    problem.solve()
    
    x = x.value
    u = u.value

    return x, u