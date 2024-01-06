import pybullet as p
import time
import pybullet_data
import math
import random

PI=math.pi
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

inner_radius = 2.5
outer_radius = 0.5
planeId = p.loadURDF("plane.urdf")

StartPos_robot = [0,0,0.8]
StartOrientation_robot = p.getQuaternionFromEuler([PI/2,0,0])

R = random.uniform(inner_radius,outer_radius)
T = random.uniform(0,2*PI)
StartPos_cube = [R*math.cos(T),R*math.sin(T),0.5]
StartOrientation_cube = p.getQuaternionFromEuler([0,0,0])

robotID = p.loadURDF("2_R_robot.urdf",StartPos_robot, StartOrientation_robot,useFixedBase=1)
cubeID = p.loadURDF("Box.urdf",StartPos_cube, StartOrientation_cube)


theta = 0

while theta<=2*PI:
    point = [inner_radius*math.cos(theta),inner_radius*math.sin(theta),0]
    p.addUserDebugPoints([point],pointColorsRGB=[[150,0,0]])
    theta+=PI/180

theta=0
while theta<=2*PI:
    point = [outer_radius*math.cos(theta),outer_radius*math.sin(theta),0]
    p.addUserDebugPoints([point],pointColorsRGB=[[0,250,0]])
    theta+=PI/180

s

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()