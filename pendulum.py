import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from control.matlab import place


guiFlag = False
dt = 1 / 240
th0 = 0.1
thd = 0.0

g = 10
L = 0.8
m = 1

A = np.array([[0, 1],
              [g / L, 0]])
B = np.array([[0],
              [1 / (m * L * L)]])

poles = np.array([-5, -6])

K = -place(A, B, poles)

physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf.xml", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=th0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

maxTime = 5
logTime = np.arange(0, maxTime, dt)
sz = len(logTime)
logThetaSim = np.zeros(sz)
logVelSim = np.zeros(sz)
logTauSim = np.zeros(sz)
idx = 0

for t in logTime:
    th, vel, _, _ = p.getJointState(boxId, 1)
    logThetaSim[idx] = th
    logVelSim[idx] = vel

    tau = K[0, 0] * th + K[0, 1] * vel
    logTauSim[idx] = tau

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=tau, controlMode=p.TORQUE_CONTROL)
    p.stepSimulation()

    idx += 1
    if guiFlag:
        time.sleep(dt)

p.disconnect()

plt.figure(figsize=(10, 8))

plt.subplot(3,1,1)
plt.plot(logTime, logThetaSim, 'b', label="Sim Pos")
plt.plot([logTime[0], logTime[-1]], [thd, thd], 'r--', label="Ref Pos")
# plt.plot(logTime, logThetaInt, 'r', label="Int Pos")
plt.grid(True)
plt.legend()

plt.subplot(3,1,2)
plt.plot(logTime, logVelSim, 'b', label="Sim Vel")
plt.grid(True)
plt.legend()

plt.subplot(3,1,3)
plt.plot(logTime, logTauSim, 'b', label="Sim Tau")
plt.grid(True)
plt.legend()
plt.show()
