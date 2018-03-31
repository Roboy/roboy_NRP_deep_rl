# roboy_NRP_deep_rl
Deep reinforcement learning for controlling Myorobotics muscles in the `Neurorobotics Platform`


This assumes you are running without gpu support. 
Dependencies

install NRP https://bitbucket.org/hbpneurorobotics/neurorobotics-platform

this may take a few hours - mind the advice on not using pip package versions

create virtual environment for deep rl packages 
avoid using sudo to pip install packages in the venv

install keras-rl https://github.com/keras-rl/keras-rl

HDF5 and h5py (required to save Keras models to disk).

install tensorflow https://www.tensorflow.org/install/install_linux
If you wish to use GPU support you may need to install 
cuDNN (recommended if you plan on running Keras on GPU).


----
workaround - potential protobuf error with NRP

pip uninstall protobuf 3.5 
pip install protobuf 3.4

https://virtualenv.pypa.io/en/stable/

add myoarm folder with files to the NRP/Experiments folder.  
This has the .exd file which defines the environment and loads the model of the myoarm, which is located in 
NRP/Models/myoarm_nst in the install
to speed up development changed the maturity attribute to 'production' so that the experiment shows up in the main NRP Browser window

.bibi file used to add the transfer functions, amongst other things

change the path to that of the virtual env in initDRLagent.py the corresponding tutorial for using tensorflow in the nrp can be found here https://developer.humanbrainproject.eu/docs/projects/HBP%20Neurorobotics%20Platform/1.2/nrp/tutorials/tensorflow/tutorial.html#installing-tensorflow-for-use-in-the-nrp
 
if you are not familiar with keras-rl, there are some parameters to set in the initDRLagent


The DDPG Agent
keras-rl provides a DDPG (Deep Deterministic Policy Gradient) agent which use two function approximating neural nets- actor and critic.  It is designed to be trained It requires the following information from the NRP Simulation to intereact with it.  

Observation is composed joint angles and angular velocity of both joints.  

The shapes of both inputs and outputs are specified in nb_actions nb_obs

The gazebo plugin used for the myoarm 2 DOF model does not publish the joint information so the gazebo file must be replaced with the given.  Be aware that this plugin assumes the use of the myoarm model because it uses string names of joints to get information from the simulation. Making this more general is a work in progress.


The actor takes the observation from the nrp as input composed of joint angles and outputs an action - 4 muscle commands. 

The critics takes the action and observation as an input and outputs an state value estimate.  

The networks are trained based upon the differences between the actual reward experienced and reward it predicts, plus a factor of entropy for exploration. 

explain reward - inverted pendulum 

The agent bookkeeps with the Sequential Memory so its important to have enough declared.  

you may start the nrp as directed in the link provided above





 

