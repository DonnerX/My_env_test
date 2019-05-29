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

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from env.neobotixschunkGymEnv import NeobotixSchunkGymEnv

def main():
    environment = NeobotixSchunkGymEnv(renders=True, isDiscrete=False, maxSteps=1e5, action_dim=9, colliObj=1, wsboundary=1)
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


########init########
    done = 0

    t=0
    disc_total_rew=0
    #
    Uid = environment._get_uid_for_test()
    state, reward, done = [0,0,0], 0, 0
    pos_z = []

    num_joint = environment._p.getNumJoints(Uid)
    for i in range(num_joint):
        print(environment._p.getJointInfo(Uid,i))
    environment._p.enableJointForceTorqueSensor(Uid,6,1)
    joint_force_fx = []
    joint_force_fy = []
    joint_force_Mx = []
    joint_force_My = []
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

            joint_state = environment._p.getJointState(Uid, 6)
            print(joint_state[2])
            joint_force_fx.append(joint_state[2][0])
            joint_force_fy.append(joint_state[2][1])
            joint_force_Mx.append(joint_state[2][2])
            joint_force_My.append(joint_state[2][3])

            disc_total_rew += 1 * 0.99 ** t
            t += 1
        else:
            pass
            #print(state)
        environment._p.removeUserDebugItem(TextId)
    print(disc_total_rew, t)
    pos_z = np.array(pos_z)
    joint_force_Mx = np.array(joint_force_Mx)
    joint_force_My = np.array(joint_force_My)
    joint_force_fx = np.array(joint_force_fx)
    joint_force_fy = np.array(joint_force_fy)
    step = np.arange(0, len(pos_z), 1)
    plt.subplot(121)
    plt.plot(step, pos_z,'r')
    plt.subplot(122)
    #plt.plot(step, joint_force_Mx, 'g')
    #plt.plot(step, joint_force_My, 'b')
    plt.plot(step, joint_force_fx,label='fx')
    plt.plot(step, joint_force_fy,label='fy')
    plt.legend()
    plt.show()



if __name__=="__main__":
    main()

