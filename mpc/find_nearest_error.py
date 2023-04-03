import numpy as np

def find_nearest_error(ref_path, robot_state):
    '''
    Finding the nearest point on reference path,
    based on the robot current state and position.
    '''
    N = 6                           # Number
    size = len(ref_path.x_ref)      # length of ref_path

    dx = []
    dy = []
    dz = []
    for i in range(size - N):
        dx.append(robot_state.x - ref_path.x_ref[i])
        dy.append(robot_state.y - ref_path.y_ref[i])
        dz.append(robot_state.z - ref_path.z_ref[i])

    distance = np.hydot(np.hydot(dx, dy), dz)
    num = int(np.argmin(distance))

    vec = np.array(dx[num], dy[num], dz[num])

    return num, vec

