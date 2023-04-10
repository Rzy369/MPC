import numpy as np
import math
import find_nearest_index

def get_ref_state(ref_path, robo_state):
    N = 6                           #[]             # Predict Horizon
    size = len(ref_path.x_ref)      # length of ref_path
    print(size)
    x_ref = np.zeros((7, N+1))
    index = find_nearest_index.find_nearest_index(ref_path, robo_state)

    print("the closest index is @ ", index)
    if index != 0 & index != size - N - 1:
        for i in range(N + 1):
            x_ref[0, i] = ref_path.x_ref[index + i]
            x_ref[1, i] = ref_path.y_ref[index + i]
            x_ref[2, i] = ref_path.z_ref[index + i]

            alpha_ref, beta_ref, gamma_ref = sol_direction(ref_path, index + i)

            x_ref[3, i] = alpha_ref
            x_ref[4, i] = beta_ref
            x_ref[5, i] = gamma_ref

            v_ref = sol_velocity(ref_path, index + i)

            x_ref[6, i] = v_ref

    if index == 0:
        x_ref[0, 0] = ref_path.x_ref[0]
        x_ref[1, 0] = ref_path.y_ref[0]
        x_ref[2, 0] = ref_path.z_ref[0]
        x_ref[3, 0] = 0
        x_ref[4, 0] = 0
        x_ref[5, 0] = 0
        x_ref[6, 0] = 0

        for i in range(1, N+1):
            x_ref[0, i] = ref_path.x_ref[index + i]
            x_ref[1, i] = ref_path.y_ref[index + i]
            x_ref[2, i] = ref_path.z_ref[index + i]

            alpha_ref, beta_ref, gamma_ref = sol_direction(ref_path, index + i)

            x_ref[3, i] = alpha_ref
            x_ref[4, i] = beta_ref
            x_ref[5, i] = gamma_ref

            v_ref = sol_velocity(ref_path, index + i)

            x_ref[6, i] = v_ref

    if index == size - N - 1:
        for i in range(N):
            x_ref[0, i] = ref_path.x_ref[index + i]
            x_ref[1, i] = ref_path.y_ref[index + i]
            x_ref[2, i] = ref_path.z_ref[index + i]

            alpha_ref, beta_ref, gamma_ref = sol_direction(ref_path, index + i)

            x_ref[3, i] = alpha_ref
            x_ref[4, i] = beta_ref
            x_ref[5, i] = gamma_ref

            v_ref = sol_velocity(ref_path, index + i)

            x_ref[6, i] = v_ref

        x_ref[0, N+1] = ref_path.x_ref[size - 1]
        x_ref[1, N+1] = ref_path.y_ref[size - 1]
        x_ref[2, N+1] = ref_path.z_ref[size - 1]
        x_ref[3, N+1] = 0
        x_ref[4, N+1] = 0
        x_ref[5, N+1] = 0
        x_ref[6, N+1] = 0
    
    # print(x_ref)
    return x_ref


def sol_direction(ref_path, index):
    x_cur = ref_path.x_ref[index]
    y_cur = ref_path.y_ref[index]
    z_cur = ref_path.z_ref[index]

    x_pre = ref_path.x_ref[index-1]
    y_pre = ref_path.y_ref[index-1]
    z_pre = ref_path.z_ref[index-1]

    x_lat = ref_path.x_ref[index+1]
    y_lat = ref_path.y_ref[index+1]
    z_lat = ref_path.z_ref[index+1]

    dx = (x_pre + x_lat - 2 * x_cur)
    dy = (y_pre + y_lat - 2 * y_cur)
    dz = (z_pre + z_lat - 2 * z_cur)

    if dz != 0 and dx != 0:
        alpha_ref = math.atan(dy/dz)
        beta_ref = math.atan(dx/dz)
        gamma_ref = math.atan(dy/dx)

    if dz == 0 and dx != 0:
        gamma_ref = math.atan(dy/dx)
        alpha_ref = 0
        beta_ref = 0

    if dz != 0 and dx == 0:
        alpha_ref = math.atan(dy/dz)
        beta_ref = math.atan(dx/dz)
        gamma_ref = 0
    
    if dz == 0 and dx == 0:
        alpha_ref = 0
        beta_ref = 0
        gamma_ref = 0

    return alpha_ref, beta_ref, gamma_ref

def sol_velocity(ref_path, index):

    dt = 0.1        #[s]                    # Time step

    x_cur = ref_path.x_ref[index]
    y_cur = ref_path.y_ref[index]
    z_cur = ref_path.z_ref[index]

    x_pre = ref_path.x_ref[index-1]
    y_pre = ref_path.y_ref[index-1]
    z_pre = ref_path.z_ref[index-1]

    x_lat = ref_path.x_ref[index+1]
    y_lat = ref_path.y_ref[index+1]
    z_lat = ref_path.z_ref[index+1]

    dis_pre = ((x_cur - x_pre) ** 2 + (y_cur - y_pre) ** 2 + (z_cur - z_pre) ** 2) ** 0.5
    dis_lat = ((x_lat - x_cur) ** 2 + (y_lat - y_cur) ** 2 + (z_lat - z_cur) ** 2) ** 0.5

    v_ref = (dis_pre + dis_lat) / (2 * dt)

    return v_ref