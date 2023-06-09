import numpy as np
import find_nearest_index

def ref_state(ref_path, robo_state, N):
    size = len(ref_path.z_ref)      # length of ref_path
    print(size)
    x_ref = np.zeros((8, N+1))
    index = find_nearest_index.find_nearest_index(ref_path, robo_state, N)
    
    print("the closest index is @ ", index)
    if index <= size - N - 1:
        for i in range(N + 1):
            x_ref[0, i] = ref_path.x_ref[index + i]
            x_ref[1, i] = ref_path.y_ref[index + i]
            x_ref[2, i] = ref_path.z_ref[index + i]
            x_ref[3, i] = ref_path.alpha_ref[index + i]
            x_ref[4, i] = ref_path.beta_ref[index + i]
            x_ref[5, i] = ref_path.gamma_ref[index + i]
            x_ref[6, i] = ref_path.v_ref[index + i]
            x_ref[7, i] = ref_path.w_ref[index + i]
        
    if index >= size - N: 
        for i in range(size - index):
            x_ref[0, i] = ref_path.x_ref[index + i]
            x_ref[1, i] = ref_path.y_ref[index + i]
            x_ref[2, i] = ref_path.z_ref[index + i]
            x_ref[3, i] = ref_path.alpha_ref[index + i]
            x_ref[4, i] = ref_path.beta_ref[index + i]
            x_ref[5, i] = ref_path.gamma_ref[index + i]
            x_ref[6, i] = ref_path.v_ref[index + i]
            x_ref[7, i] = ref_path.w_ref[index + i]
        for i in range(size-index, N):
            x_ref[0, i] = ref_path.x_ref[size - 1]
            x_ref[1, i] = ref_path.y_ref[size - 1]
            x_ref[2, i] = ref_path.z_ref[size - 1]
            x_ref[3, i] = ref_path.alpha_ref[size - 1]
            x_ref[4, i] = ref_path.beta_ref[size - 1]
            x_ref[5, i] = ref_path.gamma_ref[size - 1]
            x_ref[6, i] = 0
            x_ref[7, i] = 0
    # print("the ref state is ", x_ref)

    if index >= size - 2: 
        for i in range(N+1):
            x_ref[0, i] = ref_path.x_ref[size - 1]
            x_ref[1, i] = ref_path.y_ref[size - 1]
            x_ref[2, i] = ref_path.z_ref[size - 1]
            x_ref[3, i] = ref_path.alpha_ref[size - 1]
            x_ref[4, i] = ref_path.beta_ref[size - 1]
            x_ref[5, i] = ref_path.gamma_ref[size - 1]
            x_ref[6, i] = 0
            x_ref[7, i] = 0

        
    return x_ref