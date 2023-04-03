import numpy as np
import robot_state
import mpc
import needle_model
import path
import visualization

path = path.setting_easy_path(0.5, np.radians(20))

visualization.path_plot(path)

robo = robot_state.robot_state(x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0)

model = needle_model.needle_model_for_prediction()

x_pred, u_pred = mpc.mpc_controller(path, robo, model)
print(x_pred)