def find_nearest_index(ref_path, robo_state):
    '''
    Finding the nearest point on reference path,
    based on the robot current state and position.
    '''
    N = 6                          # Number
    size = len(ref_path.x_ref)      # length of ref_path
    err_list = []
    for i in range(1, size - N):
        dx = robo_state.x - ref_path.x_ref[i]
        dy = robo_state.y - ref_path.y_ref[i]
        dz = robo_state.z - ref_path.z_ref[i]
        err_cur = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        err_list.append(err_cur)
    err = 10000
    index = 0
    for i in range(len(err_list)):
        if err > err_list[i]:
            index = i
            err = err_list[i]

    return index
