import pybullet as p
import pybullet_data
import os
import time
import math

p.connect(p.GUI)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF("robot_arm.urdf")
p.resetBasePositionAndOrientation(robot, [0, 0, 0.05], [0, 0, 0, 0.707])

p.setGravity(0, 0, 0)

p.setJointMotorControl2(bodyIndex=robot,
                        jointIndex=0,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=2,
                        force=500)

p.setJointMotorControl2(bodyIndex=robot,
                        jointIndex=1,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=4,
                        force=500)
for i in range(100000000):
    p.stepSimulation()
