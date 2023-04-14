import numpy as np
import math
import find_nearest_index

def ref_state(ref_path, robo_state):
    N = 6                          #[]             # Predict Horizon
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
            x_ref[3, i] = ref_path.alpha_ref[index + i]
            x_ref[4, i] = ref_path.beta_ref[index + i]
            x_ref[5, i] = ref_path.gamma_ref[index + i]
            x_ref[6, i] = ref_path.v_ref[index + i]

    if index == 0:
        x_ref[0, 0] = ref_path.x_ref[0]
        x_ref[1, 0] = ref_path.y_ref[0]
        x_ref[2, 0] = ref_path.z_ref[0]
        x_ref[3, 0] = 0
        x_ref[4, 0] = 0
        x_ref[5, 0] = 0
        x_ref[6, 0] = 0
        
    if index == size - N - 1:
        for i in range(N):
            x_ref[0, i] = ref_path.x_ref[index + i]
            x_ref[1, i] = ref_path.y_ref[index + i]
            x_ref[2, i] = ref_path.z_ref[index + i]
            x_ref[3, i] = ref_path.alpha_ref[index + i]
            x_ref[4, i] = ref_path.beta_ref[index + i]
            x_ref[5, i] = ref_path.gamma_ref[index + i]
            x_ref[6, i] = ref_path.v_ref[index + i]

        x_ref[0, N] = ref_path.x_ref[size - 1]
        x_ref[1, N] = ref_path.y_ref[size - 1]
        x_ref[2, N] = ref_path.z_ref[size - 1]
        x_ref[3, N] = 0
        x_ref[4, N] = 0
        x_ref[5, N] = 0
        x_ref[6, N] = 0
        
    return x_ref