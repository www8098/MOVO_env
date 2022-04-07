import pybullet as p
import pybullet_data as pd
from pybullet_tools.ikfast.ikfast import get_ik_joints, check_ik_solver
from pybullet_tools.movo_constants import MOVO_INFOS, RIGHT, LEFT, TOOL_LINK
from pybullet_tools.utils import link_from_name
import sys
import pybullet_data
import numpy as np
import time
from Camera import *
import math


def safe_zip(sequence1, sequence2): # TODO: *args
    sequence1, sequence2 = list(sequence1), list(sequence2)
    assert len(sequence1) == len(sequence2)
    return list(zip(sequence1, sequence2))


def set_joint_position(body, joint, value):
    # TODO: remove targetVelocity=0
    p.resetJointState(body, joint, targetValue=value, targetVelocity=0)


def set_joint_positions(body, joints, values):
    for joint, value in safe_zip(joints, values):
        set_joint_position(body, joint, value)



physicsClient = p.connect(p.GUI)

# close the render
# physicsClient = p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
# 设置环境重力加速度
p.setGravity(0, 0, -10)

# 加载URDF模型，此处是加载蓝白相间的陆地
shift = [0, 3, 0]
meshScale = [0.1, 0.1, 0.1]

p.loadURDF("plane.urdf")
# p.loadURDF("samurai.urdf")
# p.loadSDF("stadium.sdf")


# add obstacles
shift = [0, 0, 0]
meshScale = [1, 1, 1]
# visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
#                                     fileName="stone.obj",
#                                     rgbaColor=[1, 1, 1, 1],
#                                     specularColor=[0.4, .4, 0],
#                                     visualFramePosition=shift,
#                                     meshScale=meshScale)
# collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
#                                           fileName="stone.obj",
#                                           collisionFramePosition=shift,
#                                           meshScale=meshScale)
#
# rangex = 5
# rangey = 5
# for i in range(rangex):
#   for j in range(rangey):
#     p.createMultiBody(baseMass=1,
#                       baseInertialFramePosition=[0, 0, 0],
#                       baseCollisionShapeIndex=collisionShapeId,
#                       baseVisualShapeIndex=visualShapeId,
#                       basePosition=[((-rangex / 2) + i) * meshScale[0] * 2,
#                                     (-rangey / 2 + j) * meshScale[1] * 2, 0],
#                       useMaximalCoordinates=True)


startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
doorPos = [5, 5, 0]
doorOrientation = p.getQuaternionFromEuler([0, 0, 0])

# p.loadSDF("stadium.sdf")
robot= p.loadURDF(".\models\movo_description\movo_robotiq_collision.urdf", startPos, startOrientation)
# door = p.loadURDF(".\models\door.urdf", doorPos, doorOrientation)
p.resetBasePositionAndOrientation(robot, startPos, startOrientation)


joints_force = [1, 1, 1, 1, 1, 1, 1, 1]


# extract the right arm joints
arm = RIGHT
ik_info = MOVO_INFOS[arm]
tool_link = link_from_name(robot, TOOL_LINK.format(arm))

check_ik_solver(ik_info)
right_ik_joints = get_ik_joints(robot, ik_info, tool_link)
right_ik_joints.pop(0)


# extract the left arm joints
arm = LEFT
ik_info = MOVO_INFOS[arm]
tool_link = link_from_name(robot, TOOL_LINK.format(arm))

check_ik_solver(ik_info)
left_ik_joints = get_ik_joints(robot, ik_info, tool_link)
left_ik_joints.pop(0)


# extract the chassis joints
chassis_ik_joints = [2, 3, 4, 5]
camera_ik_joints = [6, 7, 8]


# set control bars
right_arm_bar = []
for i in range(len(right_ik_joints)):
    right_arm_bar.append(p.addUserDebugParameter("rightArm-{}".format(str(i)), -1.5, 1.5, 0))

left_arm_bar = []
for i in range(len(left_ik_joints)):
    left_arm_bar.append(p.addUserDebugParameter("leftArm-{}".format(str(i)), -1.5, 1.5, 0))

chassis_bar = []
for i in range(len(chassis_ik_joints)):
    chassis_bar.append(p.addUserDebugParameter("chassis-{}".format(str(i)), -1.5, 1.5, 0))

max_torque = 100

# print(ik_info)
# print for debug
print(right_ik_joints)
# print(p.getNumJoints(robot))

while 1:
    # for motor in motors:
    #     p.setJointMotorControl2(robot, motor, p.POSITION_CONTROL, targetPosition=p.readUserDebugParameter(joints[motor]), force=1)

    right_arm_pos = []
    for i in range(len(right_ik_joints)):
        right_arm_pos.append(p.readUserDebugParameter(right_arm_bar[i]))

    left_arm_pos = []
    for i in range(len(left_ik_joints)):
        left_arm_pos.append(p.readUserDebugParameter(left_arm_bar[i]))

    chassis_pos = []
    for i in range(len(chassis_ik_joints)):
        chassis_pos.append(p.readUserDebugParameter(chassis_bar[i]))

    # right_ik_joints[1:] remove the lifting mechanism
    p.setJointMotorControlArray(bodyIndex=robot,
                                jointIndices=right_ik_joints,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=right_arm_pos,
                                forces=[max_torque for i in range(len(right_arm_pos))])
    # set_joint_positions(robot, right_ik_joints, right_arm_pos)
    # set_joint_positions(robot, left_ik_joints, left_arm_pos)
    # set_joint_positions(robot, chassis_ik_joints, chassis_pos)
    # setCameraPicAndGetPic(robot)

    # p.setJointMotorControlArray(bodyIndex=robot, jointIndices=right_ik_joints, controlMode=p.POSITION_CONTROL,
    #                             targetVelocities=jointsPos, forces=joints_force)

    p.stepSimulation()
    time.sleep(1 / 100)
