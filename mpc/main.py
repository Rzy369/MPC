import numpy as np
import robot_state
import mpc
import needle_model
import path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import get_ref_state
import lqr1
import ref_state

dt = 0.1        #[s]                    # Time step
time = 0        #[s]                    # Total time
k = 1/150000

# Various path to generate
# path = path.straight_path(0.5, k)
# path = path.curving_path_initial0(0.5, np.deg2rad(1), k)
path = path.curving_path_initial0(0.5, np.deg2rad(1), k)
# path = path.varying_theta_path(0.5, k)

robo_mpc = robot_state.robot_state(x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0)
robo_lqr = robot_state.robot_state(x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0)
model = needle_model.needle_model_for_prediction()

# Generating the animation of robot moving
fig = plt.figure()
ax = Axes3D(fig)
ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
ax.plot(path.x_ref, path.y_ref, path.z_ref)

# ax = fig.add_subplot(111, projection='3d')
# ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
# ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
# ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
# ax.plot(path.x_ref, path.y_ref, path.z_ref)
# plt.ion()

# print(path.x_ref)
# print(path.y_ref)
# print(path.z_ref)

x_robot = []
y_robot = []
z_robot = []
alpha_robot = []
beta_robot = []
gamma_robot = []
v = []
t = []

while time < 7:
    x_ref = ref_state.ref_state(path, robo_mpc)
    x_pred, u_pred = mpc.mpc_controller(x_ref, robo_mpc, model, k, path)
    a = u_pred[0][0]
    theta = u_pred[1][0]
    robo_mpc.state_update(a, theta, k)

    t.append(time)

    time += dt

    print("`````````````mpc```````````````````")
    print("time: ", time, " : a = ", a, " , theta = ", theta)

    x_cur = robo_mpc.x
    x_robot.append(x_cur)
    y_cur = robo_mpc.y
    y_robot.append(y_cur)
    z_cur = robo_mpc.z
    z_robot.append(z_cur)
    alpha_cur = robo_mpc.alpha
    alpha_robot.append(alpha_cur)
    beta_cur = robo_mpc.beta
    beta_robot.append(beta_cur)
    gamma_cur = robo_mpc.gamma
    gamma_robot.append(gamma_cur)
    v_cur = robo_mpc.v
    v.append(v_cur)
    

# print(x_robot)
# print(y_robot)
# print(z_robot)
# print(v)
print(alpha_robot)
print(beta_robot)
print(gamma_robot)


    # ax.scatter(x_cur, y_cur, z_cur, s=20, c='b', marker='o')
    # plt.pause(0.001)
    


    # ax1 = fig.add_subplot(111, projection='3d')
    # ax.scatter(x_ref[:, 0], x_ref[:, 1], x_ref[:, 2], x_ref[:, 3], x_ref[:, 4], x_ref[:, 5])
    
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



# Plotting the position of curve and the robot
# fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2)
# fig.suptitle('The position of robot and reference path for the path with rotation speed 0.8rad/s')
# ax1.plot(t, x_robot, 'tab:green')
# ax1.set_title("Robot's x position about time")
# ax1.set(xlabel='Time(s)', ylabel='x Position(m)')

# ax2.plot(path.x_ref, 'tab:red')
# ax2.set_title("Reference path's x position about steps")
# ax2.set(xlabel='Step', ylabel='x Position(m)')

# ax3.plot(t, y_robot, 'tab:green')
# ax3.set_title("Robot's y position about time")
# ax3.set(xlabel='Time(s)', ylabel='y Position(m)')

# ax4.plot(path.y_ref, 'tab:red')
# ax4.set_title("Reference path's y position about steps")
# ax4.set(xlabel='Step', ylabel='y Position(m)')

# ax5.plot(t, z_robot, 'tab:green')
# ax5.set_title("Robot's z position about time")
# ax5.set(xlabel='Time(s)', ylabel='z Position(m)')

# ax6.plot(path.z_ref, 'tab:red')
# ax6.set_title("Reference path's z position about steps")
# ax6.set(xlabel='Step', ylabel='z Position(m)')

# for ax in fig.get_axes():
#     ax.label_outer()

# plt.show()
