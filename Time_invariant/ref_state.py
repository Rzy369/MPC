import numpy as np
import find_nearest_index

def ref_state(ref_path, robo_state, N):
    size = len(ref_path.z_ref)      # length of ref_path
    print(size)
    x_ref = np.zeros((2, N+1))
    index = find_nearest_index.find_nearest_index(ref_path, robo_state, N)
    
    print("the closest index is @ ", index)
    if index <= size - N - 1:
        for i in range(N + 1):
            x_ref[0, i] = ref_path.z_ref[index + i]
            x_ref[1, i] = ref_path.gamma_ref[index + i]
        
    if index >= size - N: 
        for i in range(size - index):
            x_ref[0, i] = ref_path.z_ref[index + i]
            x_ref[1, i] = ref_path.gamma_ref[index + i]
        for i in range(size-index, N):
            x_ref[0, i] = ref_path.z_ref[size - 1]
            x_ref[1, i] = ref_path.gamma_ref[size - 1]

        
    return x_ref