import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

def plot_from_dict(steps,data_dict):
    for name,data in data_dict.items():
        plt.plot(steps,data,label='%s' % name)
    plt.legend()
    #plt.show()

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

#creat a cylinder
test_visual1 = p.createVisualShape(p.GEOM_CYLINDER, radius=0.5,length=0.5)
test_collision1 = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.5,height=0.5)


#creat a capsule
test_visual2 = p.createVisualShape(p.GEOM_CAPSULE, radius=0.1,length=1,
                                   visualFramePosition=[0,0,0.5],
                                   visualFrameOrientation=[0,0,0,1])
test_collision2 = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.1,height=1,
                                         collisionFramePosition=[0,0,0.5],
                                         collisionFrameOrientation=[0,0,0,1])

link_Masses = [1]
linkCollisionShapeIndices = [test_collision2]
linkVisualShapeIndices = [test_visual2]
linkPositions = [[0, 0, 0.5]]
linkOrientations = [[1, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0.5]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
parentindices = [0]
jointTypes = [p.JOINT_REVOLUTE]
axis = [[0, 1, 0]]

test_body = p.createMultiBody(baseMass=2,
                              baseCollisionShapeIndex=test_collision1,
                              baseVisualShapeIndex=test_visual1,
                              basePosition = [0, 0, 0.25],
                              linkMasses=link_Masses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=parentindices,
                              linkJointTypes=jointTypes,
                              linkJointAxis=axis)

goId = p.addUserDebugParameter('go_or_stop',0,1,1)
quitId = p.addUserDebugParameter('quit',0,1,0)
velId = p.addUserDebugParameter('vel',0,5.2,5)

num_joint = p.getNumJoints(test_body)
for i in range(num_joint):
    print(p.getJointInfo(test_body,i))

mode = p.VELOCITY_CONTROL

p.enableJointForceTorqueSensor(test_body,0,1)

joint_vel = np.array([])
joint_torq = np.array([])

base_z = np.array([])

joint_fx = np.array([])
joint_fy = np.array([])
joint_fz = np.array([])

while 1:
    go = p.readUserDebugParameter(goId)
    q = p.readUserDebugParameter(quitId)
    if q!=0:
        break
    if go:
        vel = p.readUserDebugParameter(velId)
        p.setJointMotorControl2(test_body, 0, mode, targetVelocity=vel, force=10.0)
        base_pos = p.getBasePositionAndOrientation(test_body)
        p.stepSimulation()
        joint_state = p.getJointState(test_body, 0)
        joint_vel = np.append(joint_vel,joint_state[1])
        joint_torq = np.append(joint_torq,joint_state[3])
        base_z = np.append(base_z,base_pos[0][2])
        joint_fx = np.append(joint_fx,joint_state[2][0]) #垂直杆水平
        joint_fy = np.append(joint_fy,joint_state[2][2]) ##沿着杆
        joint_fz = np.append(joint_fz, joint_state[2][1])
        time.sleep(1/120)
    else :
        print(go)

p.disconnect()

steps = range(len(joint_vel))
data_dict1 = {'joint_vel':joint_vel, 'joint_torq':joint_torq}
plt.subplot(221)
plot_from_dict(steps, data_dict1)

data_dict2 ={'base_z':base_z}
plt.subplot(223)
plot_from_dict(steps,data_dict2)

data_dict3 = {'joint_fx':joint_fx,'joint_fy':joint_fy,'joint_fz':joint_fz}
plt.subplot(222)
plot_from_dict(steps,data_dict3)

plt.show()

