import cvxpy
import numpy as np
from scipy import linalg as la
from numpy.linalg import matrix_rank

def mpc_controller(x_ref, initial_state, model, kappa, path, N):
        v0 = 0                          #[m/s]          # Initial speed
        a_max = 0.3                       #[m/s^2]        # Max acceleration
        a_min = -0.1                      #[m/s^2]        # Min acceleration
        alpha_max = np.deg2rad(1)      #[degree/s^2]             # Max rotation acceleration per time step
        alpha_min = -np.deg2rad(1)     #[degrees^2]             # Min rotation acceleration per time step
        v_max = 0.5                       #[m/s]          # maximum speed
        w_max = np.deg2rad(1)           #[degree/s]             # Max rotated speed per time step
        w_min = -np.deg2rad(1)          #[degree/s]             # Max rotated speed per time step

        Q = np.diag([1, 0.1, 0.1, 10, 10, 10])              # weighting matrix
        R = np.diag([1, 0.1])                             # weighting matrix
        P = np.diag([1000, 100, 100, 100, 100, 100])
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

        u_initial = np.array([initial_state.v, initial_state.w])

        alpha_0 = initial_state.alpha
        beta_0 = initial_state.beta
        gamma_0 = initial_state.gamma
        v_0 = initial_state.v
        w_0 = initial_state.w
        # A, B = model.sol_Matrix(alpha_0, beta_0, gamma_0, v_0, w_0, kappa)
        A, B = model.sol_Matrix(x_ref[3, 0], x_ref[4, 0], x_ref[5, 0], x_ref[6, 0], x_ref[7, 0], kappa)

        W_c = np.concatenate((B, np.dot(A,B)), axis=1)

        print('rank of the control matrix is ', matrix_rank(W_c))
        # P = la.solve_discrete_are(A, B, Q, R)

        for k in range(N+1):
                # adding the constriants
                constraints += [u[0, k] <= v_max]                      
                constraints += [u[1, k] <= w_max]
                constraints += [u[1, k] >= w_min]

                # adding the costs
                costs += cvxpy.quad_form(x[:, k] - x_ref[:6, k], Q)
                costs += cvxpy.quad_form(u[:, k], R)

        for k in range(N):
                # adding the constriants
                # constraints += [x[:, k+1] == A@x[:, k] + B@u[:, k]]
                constraints += [x[:, k+1] == A@(x[:, k] - x_ref[:6, k]) + B@(u[:, k] - x_ref[6:, k]) + x_ref[:6, k]]
                constraints += [u[0, k+1] - u[0, k] <= a_max]
                constraints += [u[0, k+1] - u[0, k] >= a_min]            
                constraints += [u[1, k+1] - u[1, k] <= alpha_max]
                constraints += [u[1, k+1] - u[1, k] >= alpha_min]


        costs += cvxpy.quad_form(x[:, N] - [path.x_ref[-1], path.y_ref[-1], path.z_ref[-1], path.alpha_ref[-1], path.beta_ref[-1], path.gamma_ref[-1]], P)
        # solving
        problem = cvxpy.Problem(cvxpy.Minimize(costs), constraints)
        problem.solve()

        # 
        x = x.value
        u = u.value

        return x, u


        
