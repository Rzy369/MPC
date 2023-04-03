import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def path_plot(path):
    fig = plt.figure()
    ax = Axes3D(fig)

    # scattering
    # ax.scatter(path.x_ref, path.y_ref, path.z_ref)

    # plotting curves
    ax.plot(path.x_ref, path.y_ref, path.z_ref)

    ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
    ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
    ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
    plt.show()