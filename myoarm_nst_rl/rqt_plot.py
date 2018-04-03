from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
@nrp.MapRobotSubscriber('joints', Topic('/jointState', JointState))

@nrp.MapRobotPublisher('x', Topic('/x', Float64))
@nrp.Robot2Neuron()
def transferfunction_0(t, joints, x):
    # log the first timestep (20ms), each couple of seconds
    if t % 2 < 0.02:
        clientLogger.info('Time: ', t)
    x.send_message(std_msgs.msg.Float64(joints.value.position[0]))
