import pybullet as p
import time
import pybullet_data

dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)
qd = 0.8
kp = 1.0
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# # go to the starting position
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
# for _ in range(1000):
#     p.stepSimulation()

# # turn off the motor for the free motion
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
while True:
    jointState = p.getJointState(boxId, 1)
    q = jointState[0]
    e = q - qd
    v = -kp*e
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=v, controlMode=p.VELOCITY_CONTROL)
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=v, controlMode=p.TORQUE_CONTROL)
    p.stepSimulation()
    time.sleep(dt)
p.disconnect()
