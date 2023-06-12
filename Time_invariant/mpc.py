import cvxpy
import numpy as np

def mpc_controller(x_ref, initial_state, model, path, N):
        v0 = 0                          #[m/s]          # Initial speed
        a_max = 0.3                       #[m/s^2]        # Max acceleration
        a_min = -0.1                      #[m/s^2]        # Min acceleration
        alpha_max = np.deg2rad(1)      #[degree/s^2]             # Max rotation acceleration per time step
        alpha_min = -np.deg2rad(1)     #[degrees^2]             # Min rotation acceleration per time step
        v_max = 0.2                       #[m/s]          # maximum speed
        w_max = np.deg2rad(5)           #[degree/s]             # Max rotated speed per time step
        w_min = -np.deg2rad(5)          #[degree/s]             # Max rotated speed per time step

        Q = np.diag([10, 0.1])              # weighting matrix
        R = np.diag([1, 1])                             # weighting matrix
        P = np.array([[37.01562119, 0], [0, 3.21267292]])

        # initializing the input and state
        x = cvxpy.Variable((2, N+1))
        u = cvxpy.Variable((2, N+1))

        # initializing the cost function and constraints
        costs = 0
        constraints = []

        # adding x = x_initial as the constraint
        x_initial = np.array([initial_state.z, initial_state.gamma])
        constraints += [x[:, 0] == x_initial]

        A, B = model.sol_Matrix()

        for k in range(N+1):
                # adding the constriants
                constraints += [u[0, k] <= v_max]                      
                constraints += [u[1, k] <= w_max]
                constraints += [u[1, k] >= w_min]

                # adding the costs
                costs += cvxpy.quad_form(x[:, k] - x_ref[:, k], Q)
                costs += cvxpy.quad_form(u[:, k], R)

                # adding terminal constriants
                constraints += [cvxpy.quad_form(x[:, N], P) <= 100]

        for k in range(N):
                # adding the constriants
                constraints += [x[:, k+1] == A@x[:, k] + B@u[:, k]]
                constraints += [u[0, k+1] - u[0, k] <= a_max]
                constraints += [u[0, k+1] - u[0, k] >= a_min]            
                constraints += [u[1, k+1] - u[1, k] <= alpha_max]
                constraints += [u[1, k+1] - u[1, k] >= alpha_min]

        costs += cvxpy.quad_form(x[:, N] - [path.z_ref[-1], path.gamma_ref[-1]], P)
        # solving
        problem = cvxpy.Problem(cvxpy.Minimize(costs), constraints)
        problem.solve()

        # 
        x = x.value
        u = u.value

        return x, u