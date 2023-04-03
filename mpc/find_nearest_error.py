import numpy as np
import sympy as sp

def find_nearest_error(ref_path, x):
    '''
    Finding the nearest point on reference path,
    based on the robot current state and position.
    '''
    N = 6                           # Number
    size = len(ref_path.x_ref)      # length of ref_path

    err = 10000
    for i in range(size - N):
        dx = x[0] - ref_path.x_ref[i]
        dy = x[1] - ref_path.y_ref[i]
        dz = x[2] - ref_path.z_ref[i]
        err_cur = np.sqrt(dx * dx + dy * dy + dz * dz)
        err = min(err_cur, err)

    return err

