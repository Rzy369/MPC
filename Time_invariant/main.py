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
N = 8                                   # Predict horizon

path = path.setting_path(0.1, np.deg2rad(0.5))

robo = robot_state.robot_state(z = 0, gamma = 0)
model = needle_model.needle_model()

x_ref = [0] * 100
y_ref = [0] * 100

# Generating the animation of robot moving
fig = plt.figure()
ax = Axes3D(fig)
ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
ax.plot(x_ref, y_ref, path.z_ref)

z_robot = []
gamma_robot = []
t = []

while time < 10:
    x_ref = ref_state.ref_state(path, robo, N)
    x_pred, u_pred = mpc.mpc_controller(x_ref, robo, model, path, N)
    v = u_pred[0][0]
    w = u_pred[1][0]
    robo.state_update(v, w)

    t.append(time)

    time += dt

    print("`````````````mpc```````````````````")
    print("time: ", time, " : v = ", v, " , w = ", w)

    z_cur = robo.z
    z_robot.append(z_cur)
    gamma_cur = robo.gamma
    gamma_robot.append(gamma_cur)


    ax.scatter(0, 0, z_cur, s=20, c='b', marker='o')
    plt.pause(0.001)

print('~~~~~~~~~~~~~~~~~~~~~~~ref z position~~~~~~~~~~~~~~~')
print(path.z_ref)

print('~~~~~~~~~~~~~~~~~~~~~~~z position of robot~~~~~~~~~~~~~~~')
print(z_robot)

print('~~~~~~~~~~~~~~~~~~~~~~~ref gamma angle~~~~~~~~~~~~~~~')
print(path.gamma_ref)

print('~~~~~~~~~~~~~~~~~~~~~~~gamma angle of robot~~~~~~~~~~~~~~~')
print(gamma_robot)