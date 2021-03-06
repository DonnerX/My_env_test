import os
import inspect
import time
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p

class joint(object):
    def __init__(self,Uid,joint_num):
        self.Uid = Uid
        self.joint_num = joint_num
        p.enableJointForceTorqueSensor(self.Uid, self.joint_num, 1)
        self.joint_force_fx = []
        self.joint_force_fy = []
        self.joint_force_fz = []
        self.joint_force_Mx = []
        self.joint_force_My = []
        self.joint_force_Mz = []
        self.joint_motor_torque = []
        self.joint_vel = []

    def append_state(self):
        self.state = p.getJointState(self.Uid, self.joint_num)
        self.joint_force_fx.append(self.state[2][0])
        self.joint_force_fy.append(self.state[2][1])
        self.joint_force_fz.append(self.state[2][2])
        self.joint_force_Mx.append(self.state[2][3])
        self.joint_force_My.append(self.state[2][4])
        self.joint_force_Mz.append(self.state[2][5])
        self.joint_motor_torque.append(self.state[3])
        self.joint_vel.append(self.state[1])

    def plot(self):
        self.joint_force_fx = np.array(self.joint_force_fx)
        self.joint_force_fy = np.array(self.joint_force_fy)
        self.joint_force_fz = np.array(self.joint_force_fz)
        self.joint_force_Mx = np.array(self.joint_force_Mx)
        self.joint_force_My = np.array(self.joint_force_My)
        self.joint_force_Mz = np.array(self.joint_force_Mz)
        self.joint_motor_torque = np.array(self.joint_motor_torque)
        self.joint_vel = np.array(self.joint_vel)
        self.steps = range(len(self.joint_force_fx))
        fig = plt.figure()
        fig.suptitle('figure of joint %s' % self.joint_num)
        ax1 = fig.add_subplot(2,2,1)
        ax1.set_title('force x,y,z')
        ax1.plot(self.steps, self.joint_force_fx, label='fx')
        ax1.plot(self.steps, self.joint_force_fy, label='fy')
        ax1.plot(self.steps, self.joint_force_fz, label='fz')
        ax1.legend()

        ax2 = fig.add_subplot(2,2,2)
        ax2.set_title('force Mx,My,Mz')
        ax2.plot(self.steps, self.joint_force_Mx, label='Mx')
        ax2.plot(self.steps, self.joint_force_My, label='My')
        ax2.plot(self.steps, self.joint_force_Mz, label='Mz')
        ax2.legend()

        ax3 = fig.add_subplot(2,2,3)
        ax3.set_title('Motor torque')
        ax3.plot(self.steps, self.joint_motor_torque,label='motor_torque')
        ax3.legend()

        ax4 = fig.add_subplot(2, 2, 4)
        ax4.set_title('joint_velocity')
        ax4.plot(self.steps, self.joint_vel, label='joint_velocity')
        ax4.legend()



currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
planeUid = p.loadURDF(os.path.join(parentdir, "My_env_test/neobotix_schunk_pybullet/data/plane.urdf"))
neobotixschunkUid = p.loadURDF(
    os.path.join(parentdir, "My_env_test/neobotix_schunk_pybullet/data/neobotixschunk/mp500lwa4d_test.urdf"),
    useFixedBase=False, flags=p.URDF_USE_SELF_COLLISION)

joint_observ1 = joint(neobotixschunkUid, 7)
joint_observ2 = joint(neobotixschunkUid, 9)
#
quitId = p.addUserDebugParameter("quit", 0, 1, 0)
#
p.setJointMotorControl2(neobotixschunkUid,9,p.POSITION_CONTROL,
                       targetPosition=0,force=50) #fz的方向为arm7旋转轴，因此arm6旋转时，会使重力在fz上的分力减小
# p.setJointMotorControl2(neobotixschunkUid,9,p.POSITION_CONTROL,
#                        targetPosition=0,force=10)   #力是表示在child_link坐标系上的，因此随着旋转fx，fy会变化
# p.setJointMotorControl2(neobotixschunkUid,9,p.POSITION_CONTROL,
#                        targetPosition=0,force=5)
# p.setJointMotorControl2(neobotixschunkUid,8,p.POSITION_CONTROL,
#                        targetPosition=0,force=5)
# p.setJointMotorControl2(neobotixschunkUid,7,p.POSITION_CONTROL,
#                        targetPosition=0,force=5)
# p.setJointMotorControl2(neobotixschunkUid,6,p.POSITION_CONTROL,
#                        targetPosition=0,force=5)
# p.setJointMotorControl2(neobotixschunkUid,5,p.POSITION_CONTROL,
#                        targetPosition=0,force=5)

num_joint = p.getNumJoints(neobotixschunkUid)
for i in range(num_joint):
    print('joint_info:', p.getJointInfo(neobotixschunkUid, i))  #joint_info return linkName of the Childlink of the joint
    print('Dynamik of links:', p.getDynamicsInfo(neobotixschunkUid, i))
    link_state = p.getLinkState(neobotixschunkUid, i)
    print('links position:', link_state[0])

pos_z = []

# while 1:
#     q = p.readUserDebugParameter(quitId)
#     if q>0:
#         break
#     p.stepSimulation()
#     joint_observ1.append_state()
#     time.sleep(1/240)

for i in range(200):
    q = p.readUserDebugParameter(quitId)
    if q>0:
        break
    p.stepSimulation()
    base_pos = p.getBasePositionAndOrientation(neobotixschunkUid)[0]
    pos_z.append(base_pos[2])
    joint_observ1.append_state()
    joint_observ2.append_state()
    time.sleep(1/240)

p.setJointMotorControl2(neobotixschunkUid,9,p.POSITION_CONTROL,
                       targetPosition=-0.5,force=50)
for i in range(200):
    q = p.readUserDebugParameter(quitId)
    if q>0:
        break
    p.stepSimulation()
    base_pos = p.getBasePositionAndOrientation(neobotixschunkUid)[0]
    pos_z.append(base_pos[2])
    joint_observ1.append_state()
    joint_observ2.append_state()
    time.sleep(1/240)

p.disconnect()

pos_z = np.array(pos_z)
step = np.arange(0, len(pos_z), 1)
plt.figure()
plt.suptitle('z position of base')
plt.plot(step, pos_z, 'r')
plt.legend()

joint_observ1.plot()
joint_observ2.plot()
plt.show()