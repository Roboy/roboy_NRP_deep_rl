"""
This module contains the transfer function which is responsible for determining the muscle movements of the myoarm
"""
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
@nrp.MapVariable("agent", initial_value=None, scope=nrp.GLOBAL)

#muscles
@nrp.MapRobotPublisher('lbm', Topic('/gazebo_muscle_interface/robot/left_bottom/cmd_activation', Float64))
@nrp.MapRobotPublisher('ltm', Topic('/gazebo_muscle_interface/robot/left_top/cmd_activation', Float64))
@nrp.MapRobotPublisher('rbm', Topic('/gazebo_muscle_interface/robot/right_bottom/cmd_activation', Float64))
@nrp.MapRobotPublisher('rtm', Topic('/gazebo_muscle_interface/robot/right_top/cmd_activation', Float64))

@nrp.MapRobotSubscriber('joints', Topic('/jointState', JointState))

@nrp.Neuron2Robot()
def controller(t, agent, lbm, ltm, rbm, rtm, joints):
    if agent.value is not None:
        clientLogger.info("FORWARD PASS")
        import math
        import numpy as np
        
        angle_lower = joints.value.position[0]
        angle_vel_lower = joints.value.velocity[0]
        angle_upper = joints.value.position[1]
        angle_vel_upper = joints.value.velocity[1]
        
        clientLogger.info('humerus_angle ', angle_lower)
        clientLogger.info('humerus_ang_vel ', angle_vel_lower)
        clientLogger.info('radius_angle ', angle_upper)
        clientLogger.info('radius_ang_vel ', angle_vel_lower)

        observation = np.array([math.cos(angle_lower), math.sin(angle_lower),
                                angle_vel_lower, math.cos(angle_upper),
                                math.sin(angle_upper), angle_vel_upper])

        # get movement action from agent and publish to robot
        action = agent.value.forward(observation)

        clientLogger.info('lbm ', action[0])
        clientLogger.info('ltm ', action[1])
        clientLogger.info('rbm ', action[2])
        clientLogger.info('rtm ', action[3])
        lbm.send_message(std_msgs.msg.Float64(action[0]))
        ltm.send_message(std_msgs.msg.Float64(action[1]))
        rbm.send_message(std_msgs.msg.Float64(action[2]))
        rtm.send_message(std_msgs.msg.Float64(action[3]))


        reward = -(angle_lower**2 + 0.1*angle_vel_lower**2 +
                   angle_upper**2 + 0.1*angle_vel_upper**2 +
                   0.001*np.sum(np.power(action, 2)))

        #learn from the reward
        agent.value.backward(reward)
        agent.value.step = agent.value.step + 1

        clientLogger.info('BACKWARD PASS, step ', agent.value.step)
        clientLogger.info('Amount of reward ', reward)
        
        if agent.value.step%20 == 0:
            clientLogger.info('saving weights')
            PATH = '/home/akshay/Documents/NRP/Experiments/myoarm_nst_rl/ddpg_weights.h5'
            agent.value.save_weights(PATH, overwrite=True)
