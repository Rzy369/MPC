from scipy import linalg as la
from numpy.linalg import matrix_rank
from numpy.linalg import inv
import numpy as np

dt = 0.1        #[s]                    # Time step

A = np.eye(2)
B = dt * np.eye(2)
Q = np.diag([10, 0.1])              # weighting matrix
R = np.diag([1, 1]) 

P = la.solve_discrete_are(A, B, Q, R)

print(P)

'''
The calculated result is 
[[37.01562119  0.        ]
 [ 0.          3.21267292]]
'''


W_c = np.concatenate((B, np.dot(A,B)), axis=1)

print(matrix_rank(W_c))

K = np.dot(np.dot(np.dot((-inv(np.dot(np.dot(np.transpose(B), P), B) + R)), np.transpose(B)), P), np.transpose(A))
A_K = A + np.dot(B, K)
print(A_K)
