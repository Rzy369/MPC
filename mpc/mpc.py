import cvxpy
import numpy as np
import needle_model

def mpc_controller(x_ref, initial_state, model):
        v0 = 0                          #[m/s]          # Initial speed
        N = 6                           #[]             # Predict Horizon
        a_max = 3                       #[m/s^2]        # Max acceleration
        a_min = -1                      #[m/s^2]        # Min acceleration
        theta_max = np.radians(60)      #[]             # Max theta per time step
        theta_min = -np.radians(60)     #[]             # Min theta per time step

        Q = np.diag([2, 2, 2, 2, 2, 2, 1])              # weighting matrix
        R = np.diag([2, 2])                             # weighting matrix

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
        A, B = model.sol_Matrix(alpha_initial, beta_initial, gamma_initial)

        for k in range(N+1):

                # adding the constriants
                constraints += [x[:, k+1] == A.dot(x[:, k]) + B.dot(u[:, k])]
                constraints += [u[0, k] <= a_max]                      
                constraints += [u[0, k] >= a_min]
                constraints += [u[1, k] <= theta_max]
                constraints += [u[1, k] >= theta_min]

                # adding the costs
                costs += cvxpy.quad_form(x[:, k] - x_ref[:, k], Q)
                costs += cvxpy.quad_form(u[:, k], R)

        # solving
        problem = cvxpy.Problem(cvxpy.Minimize(costs), constraints)
        problem.solve()

        # 
        x = x.value
        u = u.value

        return x, u


        
