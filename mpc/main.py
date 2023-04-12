import numpy as np
import robot_state
import mpc
import needle_model
import path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import get_ref_state
import lqr1

dt = 0.01        #[s]                    # Time step
time = 0        #[s]                    # Total time
k = 150

path = path.setting_easy_path(0.5, np.deg2rad(20), k)

robo_mpc = robot_state.robot_state(x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0)
robo_lqr = robot_state.robot_state(x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0)
model = needle_model.needle_model_for_prediction()

fig = plt.figure()
# ax = Axes3D(fig)
# ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
# ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
# ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
# ax.plot(path.x_ref, path.y_ref, path.z_ref)

ax = fig.add_subplot(111, projection='3d')
ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
ax.plot(path.x_ref, path.y_ref, path.z_ref)
plt.ion()

# print(path.x_ref)
# print(path.y_ref)
# print(path.z_ref)

while time < 3:
    x_ref = get_ref_state.get_ref_state(path, robo_mpc)
    x_pred, u_pred = mpc.mpc_controller(x_ref, robo_mpc, model, k, path)
    a = u_pred[0][0]
    theta = u_pred[1][0]
    robo_mpc.state_update(a, theta, k)
    time += dt

    print("`````````````mpc```````````````````")
    print("time: ", time, " : a = ", a, " , theta = ", theta)

    x_cur = robo_mpc.x
    y_cur = robo_mpc.y
    z_cur = robo_mpc.z
    ax.scatter(x_cur, y_cur, z_cur, s=20, c='b', marker='o')
    plt.pause(0.001)
    
    # x_ref = get_ref_state.get_ref_state(path, robo_lqr)
    # x_lqr, u_lqr = lqr1.lqr_controller(x_ref, robo_lqr, model, k, path)
    # a_lqr = u_lqr[0][0]
    # theta_lqr = u_lqr[1][0]
    # robo_lqr.state_update(a_lqr, theta_lqr, k)
    # print("`````````````lqr```````````````````")
    # print("time: ", time, " : a = ", a_lqr, " , theta = ", theta_lqr)
    # x_lqr = robo_lqr.x
    # y_lqr = robo_lqr.y
    # z_lqr = robo_lqr.z
    # ax.scatter(x_lqr, y_lqr, z_lqr, s=20, c='r', marker='o')
    




