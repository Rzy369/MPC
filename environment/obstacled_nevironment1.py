import time
import pybullet as p
import pybullet_data


if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

    # setting the gravity
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(1)

    # load plane urdf
    floor = p.loadURDF("plane.urdf", useMaximalCoordinates=True)

	# establishing walls
    wall_1_id = p.loadURDF("model/wall1.urdf", basePosition=[0, 1, 0], useFixedBase=True)
    wall_2_id = p.loadURDF("model/wall2.urdf", basePosition=[0, -1, 0], useFixedBase=True)

    # adding obstacles
    obstacle_1_id = p.loadURDF("model/obstacle1.urdf", basePosition=[0, -0.6, 0], useFixedBase=True)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    while True:
        time.sleep(1. / 240.)
