import cvxpy
import numpy as np
from scipy import linalg as la
from numpy.linalg import matrix_rank

def mpc_controller(x_ref, initial_state, model, kappa, path, N):
        dt = 0.1                        #s
        v0 = 0                          #[m/s]          # Initial speed
        a_max = 3                       #[m/s^2]        # Max acceleration
        a_min = -1                      #[m/s^2]        # Min acceleration
        alpha_max = np.deg2rad(1)      #[degree/s^2]             # Max rotation acceleration per time step
        alpha_min = -np.deg2rad(1)     #[degrees^2]             # Min rotation acceleration per time step
        v_max = 0.2                       #[m/s]          # maximum speed
        w_max = np.deg2rad(2)           #[degree/s]             # Max rotated speed per time step
        w_min = -np.deg2rad(2)          #[degree/s]             # Max rotated speed per time step

        Q = np.diag([0.01, 0.1, 0.1, 0.1, 0.1, 0.1])              # weighting matrix
        R = np.diag([100, 100])                             # weighting matrix
        # initializing the input and state
        x = cvxpy.Variable((6, N+1))
        u = cvxpy.Variable((2, N+1))
        
        # initializing the cost function and constraints
        costs = 0

        constraints = []

        # adding x = x_initial as the constraint
        x_initial = np.array([initial_state.x, initial_state.y, initial_state.z,
                              initial_state.alpha, initial_state.beta, initial_state.gamma])
        constraints += [x[:, 0] == x_initial]

        # A, B = model.sol_Matrix(alpha_0, beta_0, gamma_0, v_0, w_0, kappa)
        A, B = model.sol_Matrix(x_ref[3, 0], x_ref[4, 0], x_ref[5, 0], x_ref[6, 0], x_ref[7, 0], kappa)

        W_c = np.concatenate((B, np.dot(A,B)), axis=1)

        # print('rank of the control matrix is ', matrix_rank(W_c))
        P = la.solve_discrete_are(A, B, Q, R)

        # print(P)

        for k in range(N+1):
                # adding the constriants
                constraints += [u[0, k] + x_ref[6, k] <= v_max]                      
                constraints += [u[1, k] + x_ref[7, k] <= w_max]
                constraints += [u[1, k] + x_ref[7, k] >= w_min]

                # adding the costs
                costs += cvxpy.quad_form(x[:, k] - x_ref[:6, k], Q)
                costs += cvxpy.quad_form(u[:, k], R)

        for k in range(N):
                # adding the constriants
                # constraints += [x[:, k+1] == A@x[:, k] + B@u[:, k]]
                constraints += [x[:, k+1] == A@(x[:, k] - x_ref[:6, k]) + B@(u[:, k] - x_ref[6:, k]) + x_ref[:6, k]]
                constraints += [u[0, k+1] + x_ref[6, k+1] - u[0, k] - x_ref[6, k] <= a_max * dt]
                constraints += [u[0, k+1] + x_ref[6, k+1] - u[0, k] - x_ref[6, k] >= a_min * dt]            
                constraints += [u[1, k+1] + x_ref[7, k+1] - u[1, k] - x_ref[7, k] <= alpha_max * dt]
                constraints += [u[1, k+1] + x_ref[7, k+1] - u[1, k] - x_ref[7, k] >= alpha_min * dt]

        # adding terminal constriants
        constraints += [cvxpy.quad_form(x[:, N] - x_ref[:6, k], P) <= 10]

        costs += cvxpy.quad_form(x[:, N] - x_ref[:6, k], P)
        # solving
        problem = cvxpy.Problem(cvxpy.Minimize(costs), constraints)
        problem.solve()

        # 
        x = x.value
        u = u.value

        return x, u


        
