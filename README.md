# roboy_NRP_deep_rl
_Deep reinforcement learning for controlling myorobotics muscles in the Neurorobotics Platform (NRP)_


## Dependencies

Please note: the following assumes no gpu support.  Tested on Ubuntu 16.04.

Install NRP https://bitbucket.org/hbpneurorobotics/neurorobotics-platform.  This may take a few hours - mind the advice on not using pip package versions.

Create a virtual environment (venv) for installing deep reinforcement learning packages. https://virtualenv.pypa.io/en/stable/. Avoid using `sudo` to install packages in the venv.  Make sure to isolate using `--no-site-packages`.

In the venv:
* Install tensorflow https://www.tensorflow.org/install/install_linux for Keras Backend

* Install keras-rl https://github.com/keras-rl/keras-rl

* Install h5py (required to save Keras models to disk) 
`pip install h5py`

## Workaround(s) 

Protobuf Error when starting experiment in NRP:

```
pip uninstall protobuf 
pip install protobuf==3.4
```

## Experiment related files/folders

Copy `myoarm_nst_rl/` folder to the `NRP/Experiments` folder.  
* The `.exc` file which specifies the environment model and physics engine for the simulation.  To speed up development, the `<maturity>` attribute has been set to `production` so that the experiment shows up in the main set of NRP experiment templates.
* The `.bibi` file specifies the transfer functions, or python scripts executed every step of the simulation.

* In `myoarm_nst_rl/initDRLAgent.py`:
  * Change the addsite path to that of the virtual env created. You may find more details in the corresponding tutorial for using `tensorflow` in the NRP [here](https://developer.humanbrainproject.eu/docs/projects/HBP%20Neurorobotics%20Platform/1.2/nrp/tutorials/tensorflow/tutorial.html#installing-tensorflow-for-use-in-the-nrp).
  
  * Change the path from where the saved model weights should be loaded. By default this should be the experiment folder `myoarm_nst_rl/`

* In `myoarm_nst_rl/controller.py`:
  * Change the path to where the model weights should be saved.  By default this should be the experiment folder `myoarm_nst_rl/`

---

Replace `model.sdf` in the `NRP/Models/myoarm_nst/` folder.

---

In order to get joint information from the simulation, add following lines in `NRP/GazeboRosPackages/src/gazebo_ros_muscle_interface/src/gazebo_ros_muscle_interface.cpp`:

* In `void MuscleInterfacePlugin::Init()`

```
m_joint_pub = rosNode->advertise<sensor_msgs::JointState>("/jointState", 10);
```

* In `void MuscleInterfacePlugin::OnUpdateEnd(/*const common::UpdateInfo & _info*/)`
```
sensor_msgs::JointState msg;

msg.position.push_back(m_model->GetJoint("HumerusBoneJoint")->GetAngle(0).Radian());
msg.position.push_back(m_model->GetJoint("RadiusBoneJoint")->GetAngle(0).Radian());
msg.velocity.push_back(m_model->GetJoint("HumerusBoneJoint")->GetVelocity(0));
msg.velocity.push_back(m_model->GetJoint("RadiusBoneJoint")->GetVelocity(0));

m_joint_pub.publish(msg);

```
Make sure the appropriate headers are there in `NRP/GazeboRosPackages/src/gazebo_ros_muscle_interface/include/gazebo_ros_muscle_interface.h`:

```
#include <gazebo/physics/opensim/OpensimJoint.hh>
...
#include <sensor_msgs/JointState.h>

```
This assumes the use of string names in the myoarm `model.sdf`, and making this more general is a work in progress.


## Setting up D-RL Agent
 
`keras-rl` provides a DDPG (Deep Deterministic Policy Gradient) agent which uses two function approximating neural nets - actor and critic.  It is suitable for control tasks where the action space is continous, and the state space high-dimensional. The task here is similar to the canonical inverted-pendulum task for reinforcement learning agents. Although the state and action spaces may not be very large here, this experiment serves as a prototype for using keras-rl in the NRP to control more complex locomotion of Roboy's myomuscle legs.   

* Action space: 4 muscle activation commands - left top, left bottom, right top, right bottom

* State space: angle and angular velocities of both joints
(The sizes, shapes are stored in `nb_actions` and `obs_shape` in `myoarm_nst_rl/initDRLAgent.py` respectively.) 

* Reward: computed as the negative sum of the angles, angle velocities, and muscle commands, for which the maximum reward is achieved when the arm stands straight up (like the inverted pendulum) and uses minimal muscle effort to do so 

The DDPG agent actor net takes state information as input and outputs an action - the 4 muscle commands. The critics takes the action and state as an input and outputs an value function estimate.  The paper can be found [here](https://arxiv.org/abs/1509.02971).

The networks are trained w.r.t. the minimization of a loss function derived from the difference between the actual reward experienced and reward it predicts, plus a factor of action entropy for exploration.

A `keras-rl` agent usually takes the the simulation/environment as an object and updates itself while stepping through it.  In this implementation, the NRP takes the agent as an object (so to say), steps itself, and updates the agent along the way.  The roles of the stepper are reversed, as seen in the image below.  

![alt text](https://github.com/Roboy/roboy_NRP_deep_rl/blob/master/VR-DeepRLinNRP.jpg)


#### Other Agent Parameters
The agent bookkeeps with the `SequentialMemory()` so its important to have enough declared.  

`keras-rl`'s network warm-up cycles (`nb_steps_warmup_critic`, `nb_steps_warmup_actor`) are not fully employed.


## Running the experiment 
Please refer to the NRP [documentation](https://bitbucket.org/hbpneurorobotics/neurorobotics-platform) to get it up and running and to execute the experiment.  
