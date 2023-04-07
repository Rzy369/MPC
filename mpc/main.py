import numpy as np
import robot_state
import mpc
import needle_model
import path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import get_ref_state

dt = 0.1        #[s]                    # Time step
time = 0        #[s]                    # Total time


path = path.setting_easy_path(0.5, np.radians(20))

robo = robot_state.robot_state(x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0)
model = needle_model.needle_model_for_prediction()

fig = plt.figure()
ax = Axes3D(fig)
ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
ax.plot(path.x_ref, path.y_ref, path.z_ref)
plt.ion()

while time < 15:
    x_ref = get_ref_state.get_ref_state(path, robo)
    x_pred, u_pred = mpc.mpc_controller(x_ref, robo, model)
    a = u_pred[0][0]
    theta = u_pred[1][0]
    robo.state_update(a, theta)
    time += dt

    print("``````````````````````````````````````")
    print("time: ", time, " : a = ", a, " , theta = ", theta)

    x_cur = robo.x
    y_cur = robo.y
    z_cur = robo.z
    ax.scatter(x_cur, y_cur, z_cur, s=20, c='b', marker='o')
    plt.pause(0.01)




