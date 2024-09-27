import pybullet as p
import time
import pybullet_data
import numpy as np

dt = 1/240 # pybullet simulation step
q0 = 0.0   # starting position (radian)
qd = 0.8
kp = 40.0
kd = 8.0
ki = 40.0

maxTime = 10
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos = np.zeros(sz)
logPos[0] = q0
logVel = np.zeros(sz)
logCtrl = np.zeros(sz)
idx = 0
e_int = 0.0

physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# # go to the starting position
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
# for _ in range(1000):
#     p.stepSimulation()

# # turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
for t in logTime[1:]:
    jointState = p.getJointState(boxId, 1)
    q = jointState[0]
    dq = jointState[1]
    e = q - qd
    e_int += e*dt
    v = -kp*e - kd*dq - ki*e_int
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=v, controlMode=p.VELOCITY_CONTROL)
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=v, controlMode=p.TORQUE_CONTROL)
    p.stepSimulation()

    jointState = p.getJointState(boxId, 1)
    dq = jointState[1]
    logVel[idx] = dq
    logCtrl[idx] = v
    idx += 1
    logPos[idx] = q

logCtrl[-1] = v
p.disconnect()

import matplotlib.pyplot as plt

plt.subplot(2,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot([logTime[0], logTime[-1]], [qd, qd], label = "refPos")
plt.legend()

plt.subplot(2,1,2)
plt.grid(True)
plt.plot(logTime, logCtrl, label = "simCtrl")
plt.plot(logTime, logVel, label = "simVel")
plt.legend()

plt.show()
