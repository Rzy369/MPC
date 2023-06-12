import numpy as np
import robot_state
import mpc
import needle_model
import path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ref_state

dt = 0.1        #[s]                    # Time step
time = 0        #[s]                    # Total time
N = 10                                   # Predict horizon

kappa = 100/15

path = path.setting_path(0.1, np.deg2rad(0.1), kappa)

robo = robot_state.robot_state(x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0, v = 0.1, w = np.deg2rad(0.1))
model = needle_model.needle_model()

# Generating the animation of robot moving
fig = plt.figure()
ax = Axes3D(fig)
ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
ax.plot(path.x_ref, path.y_ref, path.z_ref)

x_robot = []
y_robot = []
z_robot = []
alpha_robot = []
beta_robot = []
gamma_robot = []
t = []

while time < 10:
    x_ref = ref_state.ref_state(path, robo, N)
    x_pred, u_pred = mpc.mpc_controller(x_ref, robo, model, kappa, path, N)
    v = u_pred[0][0]
    w = u_pred[1][0]
    # v = 0.1
    # w = np.deg2rad(1)
    robo.state_update(v,w,kappa)

    t.append(time)

    time += dt

    print("`````````````mpc```````````````````")
    print("time: ", time, " : v = ", v, " , w = ", w)

    x_cur = robo.x
    x_robot.append(x_cur)
    y_cur = robo.y
    y_robot.append(y_cur)
    z_cur = robo.z
    z_robot.append(z_cur)
    alpha_cur = robo.alpha
    alpha_robot.append(alpha_cur)
    beta_cur = robo.beta
    beta_robot.append(beta_cur)
    gamma_cur = robo.gamma
    gamma_robot.append(gamma_cur)


    ax.scatter(x_cur, y_cur, z_cur, s=20, c='b', marker='o')
    plt.pause(0.001)

print('~~~~~~~~~~~~~~~~~~~~~~~ref x position~~~~~~~~~~~~~~~')
print(path.x_ref)

print('~~~~~~~~~~~~~~~~~~~~~~~x position of robot~~~~~~~~~~~~~~~')
print(x_robot)

print('~~~~~~~~~~~~~~~~~~~~~~~ref y position~~~~~~~~~~~~~~~')
print(path.y_ref)

print('~~~~~~~~~~~~~~~~~~~~~~~y position of robot~~~~~~~~~~~~~~~')
print(y_robot)

print('~~~~~~~~~~~~~~~~~~~~~~~ref z position~~~~~~~~~~~~~~~')
print(path.z_ref)

print('~~~~~~~~~~~~~~~~~~~~~~~x position of robot~~~~~~~~~~~~~~~')
print(z_robot)

print('~~~~~~~~~~~~~~~~~~~~~~~ref alpha angle~~~~~~~~~~~~~~~')
print(path.alpha_ref)

print('~~~~~~~~~~~~~~~~~~~~~~~alpha angle of robot~~~~~~~~~~~~~~~')
print(alpha_robot)

print('~~~~~~~~~~~~~~~~~~~~~~~ref beta angle~~~~~~~~~~~~~~~')
print(path.beta_ref)

print('~~~~~~~~~~~~~~~~~~~~~~~beta angle of robot~~~~~~~~~~~~~~~')
print(beta_robot)

print('~~~~~~~~~~~~~~~~~~~~~~~ref gamma angle~~~~~~~~~~~~~~~')
print(path.gamma_ref)

print('~~~~~~~~~~~~~~~~~~~~~~~gamma angle of robot~~~~~~~~~~~~~~~')
print(gamma_robot)