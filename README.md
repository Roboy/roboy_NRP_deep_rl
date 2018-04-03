# roboy_NRP_deep_rl
Deep reinforcement learning for controlling myorobotics muscles in the Neurorobotics Platform (NRP)

## Dependencies

Please note: The following assumes you are running without gpu support. 
Tested on Ubuntu 16.04.

Install NRP https://bitbucket.org/hbpneurorobotics/neurorobotics-platform.
This may take a few hours - mind the advice on not using pip package versions.

Create virtual environment for deep reinforcement learning packages. https://virtualenv.pypa.io/en/stable/ 
Avoid using ' sudo ' to install packages in the venv.
Make sure to isolate using '--no-site-packages'.

In the virtual environment created:
  * Install tensorflow https://www.tensorflow.org/install/install_linux for Keras backend

  * Install keras-rl https://github.com/keras-rl/keras-rl

  * Install h5py (required to save Keras models to disk).
  'pip install h5py'

## Workaround(s) 

Protobuf Error when starting experiment in NRP

''' pip uninstall protobuf 
pip install protobuf --version 3.4 '''


## Experiment related files/folders

Add ' myoarm_nst_rl/ ' folder ' NRP/Experiments ' folder.  
The ' .exc ' file which specifies the environment model and physics engine for the simulation.  To speed up development, the ' <maturity> ' attribute has been set to ' production ' so that the experiment shows up in the main set of NRP experiment templates.
The ' .bibi ' file specifies the transfer functions, or python scripts executed every step of the simulation.

Place ' model.sdf ' in the ' NRP/Models/myoarm_nst/ ' and replace the existing file of the same name

In initDRLAgent.py:
  *Change the addsite path to that of the virtual env created. You may find more details in the corresponding tutorial for using tensorflow in the NRP [here].(https://developer.humanbrainproject.eu/docs/projects/HBP%20Neurorobotics%20Platform/1.2/nrp/tutorials/tensorflow/tutorial.html#installing-tensorflow-for-use-in-the-nrp)

  *Change the path from where the saved model weights should be loaded. By default this should be the experiment folder 'myoarm_nst_rl/'

In controller.py:
  *Change the path to where the model weights should be saved.  By default this should be the experiment folder 'myoarm_nst_rl/'

In order to get joint information from the simulation, replace 'NRP/GazeboRosPackages/src/gazebo_muscle_plugin/' with the one here. Be aware that this plugin assumes the use of the myoarm **model.sdf** and uses string names of joints to get information. Making this more general is a work in progress.


## Setting up D-RL Agent
 
keras-rl provides a DDPG (Deep Deterministic Policy Gradient) agent which uses two function approximating neural nets - actor and critic.  It is suitable for control tasks where the action space is continous, and the state space high-dimensional. The task here is similar to the canonical inverted-pendulum task for reinforcement learning agents. Although the state and action spaces may not be very large here, this experiment serves as a prototype for using keras-rl in the NRP to control more complex locomotion of Roboy's myomuscle legs.   

Action space: 4 muscle activation commands, left top, left bottom, right top, right bottom and outputs are specified in The State space: angle and angular velocities of both joints.
(The sizes, shapes are stored in ' nb_actions ' and ' obs_shape ' respectively.) 

The reward is computed as the negative sum of the angles, angle velocities, and muscle commands, for which the maximum reward is achieved when the arm stands straight up (like the inverted pendulum, and uses minimal muscle effort to do so.  

The DDPG agent actor net takes state information as input and outputs an action - the 4 muscle commands. The critics takes the action and state as an input and outputs an value function estimate.  The paper can be found [here]. (https://arxiv.org/abs/1509.02971)

The networks are trained based upon the minimization of a loss function derived from the difference between the actual reward experienced and reward it predicts, plus a factor of entropy for exploration.

A keras-rl agent usually takes the the simulation/environment as an object and updates itself while stepping through it.  In this implementation, the NRP takes the agent as an object (so to say), steps itself, and updates the agent along the way.  The roles of the stepper are reversed, as seen in this image.  

(https://github.com/Roboy/roboy_NRP_deep_rl/blob/master/VR-DeepRLinNRP.jpg)


### Other Agent Parameters
The agent bookkeeps with the 'SequentialMemory()' so its important to have enough declared.  

keras-rl's network warm-up cycles are ignored in this usage.


## Running the experiment 
Please refer to the NRP documentation to get it up and running and to run the experiment.  
