# internal keras-rl agent to persist
@nrp.MapVariable("agent", initial_value=None, scope=nrp.GLOBAL)
@nrp.Robot2Neuron()
def init_DRLagent(t, agent):
    # initialize the keras-rl agent
    if agent.value is None:
        # import keras-rl in NRP through virtual env
        import site, os
        site.addsitedir(os.path.expanduser('~/.opt/tensorflow_venv/lib/python2.7/site-packages'))
        from keras.models import Model, Sequential
        from keras.layers import Dense, Activation, Flatten, Input, concatenate
        from keras.optimizers import Adam, RMSprop
        from rl.agents import DDPGAgent
        from rl.memory import SequentialMemory
        from rl.random import OrnsteinUhlenbeckProcess

        from keras import backend as K
        
        K.clear_session()

        obs_shape = (6,)

        nb_actions = 4

        clientLogger.info('INIT AGENT')

        clientLogger.info('obs_shape', obs_shape)
        
        # create the nets for rl agent
        # actor net
        actor = Sequential()
        actor.add(Flatten(input_shape=(1,) + obs_shape))
        actor.add(Dense(32))
        actor.add(Activation('relu'))
        actor.add(Dense(32))
        actor.add(Activation('relu'))
        actor.add(Dense(32))
        actor.add(Activation('relu'))
        actor.add(Dense(nb_actions))
        actor.add(Activation('sigmoid'))

        # critic net
        action_input = Input(shape=(nb_actions,), name='action_input')
        observation_input = Input(shape=(1,) + obs_shape, name='observation_input')
        flattened_observation = Flatten()(observation_input)
        x = concatenate([action_input, flattened_observation])
        x = Dense(64)(x)
        x = Activation('relu')(x)
        x = Dense(64)(x)
        x = Activation('relu')(x)
        x = Dense(64)(x)
        x = Activation('relu')(x)
        x = Dense(1)(x)
        x = Activation('linear')(x)
        critic = Model(inputs=[action_input, observation_input], outputs=x)

        
        # instanstiate rl agent
        memory = SequentialMemory(limit=1000, window_length=1)
        random_process = OrnsteinUhlenbeckProcess(theta=.15, mu=0., sigma=.2, size=nb_actions)
        agent.value = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input, memory=memory, nb_steps_warmup_critic=10, nb_steps_warmup_actor=10, random_process=random_process, gamma=.99, batch_size=5, target_model_update=1e-3, delta_clip=1.)
        agent.value.training = True
                
        PATH = '/home/akshay/Documents/NRP/Experiments/myoarm_nst_rl/ddpg_weights.h5'
        if os.path.isfile(PATH):
            print('loading weights')
            agent.value.load_weights(PATH)
        
        agent.value.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])
        
