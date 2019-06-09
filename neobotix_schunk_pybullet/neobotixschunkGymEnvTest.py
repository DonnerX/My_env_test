'''
original built by X. Wang & Z. Zheng, @KIT-IPR
developed by Z. Zheng
schunk model meshes source : https://github.com/ipa320/schunk_modular_robotics
neobotix model meshed source : https://github.com/neobotix/neo_mp_500
model modified by Y. Zhang and J. Su.
'''
import os
import inspect
import time
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from env.neobotixschunkGymEnv import NeobotixSchunkGymEnv

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




def main():
    environment = NeobotixSchunkGymEnv(renders=True, isDiscrete=False, maxSteps=1e5, action_dim=9, colliObj=0, wsboundary=1)
    # environment._p.startStateLogging(environment._p.STATE_LOGGING_VIDEO_MP4, "TEST_GUI.mp4")
    dv = 1
    actionIds = []
    actionIds.append(environment._p.addUserDebugParameter("basevelocity", -dv, dv, 0))
    actionIds.append(environment._p.addUserDebugParameter("baseangularvelocity", -dv, dv, 0))
    actionIds.append(environment._p.addUserDebugParameter("arm_1_joint", -dv, dv, 0))
    actionIds.append(environment._p.addUserDebugParameter("arm_2_joint", -dv, dv, 0))
    actionIds.append(environment._p.addUserDebugParameter("arm_3_joint", -dv, dv, 0))
    actionIds.append(environment._p.addUserDebugParameter("arm_4_joint", -dv, dv, 0))
    actionIds.append(environment._p.addUserDebugParameter("arm_5_joint", -dv, dv, 0))
    actionIds.append(environment._p.addUserDebugParameter("arm_6_joint", -dv, dv, 0))
    actionIds.append(environment._p.addUserDebugParameter("arm_7_joint", -dv, dv, 0))
    goId = environment._p.addUserDebugParameter("stop_or_go",0,1,0)
    quitId = environment._p.addUserDebugParameter("quit",0,1,0)
    #p.addUserDebugLine([0,0,1],[0,0.5,1],[0,0,0])


########init########
    done = 0

    t=0
    disc_total_rew=0
    #
    Uid = environment._get_uid_for_test()
    state, reward, done = [0,0,0], 0, 0
    pos_z = []

    num_joint = environment._p.getNumJoints(Uid)
    print(environment._p.getDynamicsInfo(Uid, -1))
    for i in range(num_joint):
        print(environment._p.getJointInfo(Uid,i))
        print(environment._p.getDynamicsInfo(Uid,i))
        link_state = environment._p.getLinkState(Uid,i)
        print(link_state[0])
    joint_observ = joint(Uid,6)
####################

    while not done:
        #time.sleep(1)
        #environment.reset()
        TextId = environment._p.addUserDebugText("reward=%.2f,done=%s" % (reward, done), [0, 0, 1
                                                                                          ], [0, 0, 0])
        go = environment._p.readUserDebugParameter(goId)
        q = environment._p.readUserDebugParameter(quitId)
        if q>0:
            environment._p.disconnect()
            break
        if go:
            action = []
            for actionId in actionIds:
                action.append(environment._p.readUserDebugParameter(actionId))
            state, reward, done, info = environment.step(action)
            #state, reward, done, info = environment.step(environment._sample_action())
            # print('r', reward)
            #print('step', state, reward, done, info)
            obs = environment.getExtendedObservation()
            # print(environment._p.getPhysicsEngineParameters)
            # environment.render()
            base_pos = environment._p.getBasePositionAndOrientation(Uid)[0]
            pos_z.append(base_pos[2])

            joint_observ.append_state()

            disc_total_rew += 1 * 0.99 ** t
            t += 1
        else:
            pass
            #print(state)
        environment._p.removeUserDebugItem(TextId)
    print(disc_total_rew, t)

    pos_z = np.array(pos_z)
    step = np.arange(0, len(pos_z), 1)
    plt.figure()
    plt.suptitle('z position of base')
    plt.plot(step, pos_z,'r')
    plt.legend()
    joint_observ.plot()
    plt.show()



if __name__=="__main__":
    main()

