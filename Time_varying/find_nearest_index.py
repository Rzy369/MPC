import numpy as np

def find_nearest_index(ref_path, robo_state, N):
    '''
    Finding the nearest point on reference path,
    based on the robot current state and position.
    '''
    size = len(ref_path.z_ref)      # length of ref_path
    err_list = []
    for i in range(0, size):
        dx = abs(robo_state.x - ref_path.x_ref[i])
        dy = abs(robo_state.y - ref_path.y_ref[i])
        dz = abs(robo_state.z - ref_path.z_ref[i])
        err_cur = np.sqrt(dx**2 + dy**2 + dz**2)
        err_list.append(err_cur)
    index = np.argmin(err_list)

    return index