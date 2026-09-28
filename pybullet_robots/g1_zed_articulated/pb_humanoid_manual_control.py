import pybullet as p
import time

import pybullet_data

p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#obUids = p.loadMJCF("g1_29dof_with_hand_rev_1_0.xml")
#humanoid = p.loadURDF("g1_29dof_with_hand_zed_mount.urdf", useFixedBase=True)
humanoid = p.loadURDF("g1_29dof_with_hand_aruco_zed_mount_surgeleft.urdf", useFixedBase=True)#g1_29dof_with_hand_zed_mount_surgeleft.urdf", useFixedBase=True)
#humanoid = obUids[1]

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(humanoid, -1, linearDamping=0, angularDamping=0)

for j in range(p.getNumJoints(humanoid)):
  p.changeDynamics(humanoid, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(humanoid, j)
  #print(info)
  jointName = info[1]
  print("jointName=",jointName)
  if jointName=="left_hand_thumb_0_joint":
    print("jointName!")
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))

p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = p.readUserDebugParameter(c)
    p.setJointMotorControl2(humanoid, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
  time.sleep(0.01)
