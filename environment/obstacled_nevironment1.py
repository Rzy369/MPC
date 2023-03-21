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


    #     shapeType = p.GEOM_MESH,
    #     fileName = "cube.obj",
    #     rgbaColor = [1, 1, 1, 1],
    #     specularColor = [0.4, 0.4, 0],
    #     visualFramePosition = [0, 0, 0],
    #     meshScale = [1, 1, 1])

	# # 碰撞属性
    # collision_ind = p.createCollisionShape(
    #     shapeType = p.GEOM_MESH,
    #     fileName = "cube.obj",
    #     collisionFramePosition = [0, 0, 0],
    #     meshScale = [1, 1, 1])
	
	# # 从视觉和碰撞属性中创建模型
    # p.createMultiBody(
    #     baseMass = 1,
    #     baseCollisionShapeIndex = collision_ind,
    #     baseVisualShapeIndex = visual_ind,
    #     basePosition = [0, 0, 1],
    #     useMaximalCoordinates = True)



    # establishing a wall
    # wall_collison_1 = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[10, 0.5, 0.5])
    # wall_visual_1 = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[10, 0.5, 0.5])
    # wall_1 = p.createMultiBody(baseMass=100, baseCollisionShapeIndex=wall_collison_1,
    #                         baseVisualShapeIndex=wall_visual_1, basePosition=[0, 0, 0])

    # p.createConstraint(floor, -1, wall_1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 2], [0, 0, 1])

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    while True:
        time.sleep(1. / 240.)
