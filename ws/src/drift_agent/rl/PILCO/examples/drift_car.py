import numpy as np
import gym
import drift_gym
import gpflow
from pilco.models import PILCO
from pilco.controllers import RbfController, LinearController
from pilco.rewards import ExponentialReward
import tensorflow as tf
from tensorflow import logging
from utils import rollout, policy
np.random.seed(0)

# Introduces a simple wrapper for the gym environment
# Reduces dimensions, avoids non-smooth parts of the state space that we can't model
# Uses a different number of timesteps for planning and testing
# Introduces priors


class DriftCarWrapper():
    def __init__(self):
        self.env = gym.make('DriftCarGazeboContinuousBodyFrame4WD-v1')
        self.action_space = self.env.action_space
        self.observation_space = self.env.observation_space

    def step(self, action):
        ob, r, done, _ = self.env.step(action)
        return ob, r, done, {}

    def reset(self):
        ob = self.env.reset()
        return ob

    def render(self):
        self.env.render()


SUBS = 1
bf = 10
maxiter = 10
state_dim = 3
control_dim = 1
max_action = 0.8 # actions for these environments are discrete
target = np.array([-3.5, 0.5, 2])
weights = np.eye(state_dim)
# weights[0,0] = 0.5
# weights[3,3] = 0.5
m_init = np.zeros(state_dim)[None, :]
S_init = 0.01 * np.eye(state_dim)
T = 100
J = 7
N = 15
T_sim = 100
restarts=True
lens = []

with tf.Session() as sess:
    env = DriftCarWrapper()

    # Initial random rollouts to generate a dataset
    X,Y = rollout(env, None, timesteps=T, random=True, SUBS=SUBS)
    for i in range(1,J):
        X_, Y_ = rollout(env, None, timesteps=T, random=True, SUBS=SUBS, verbose=True)
        X = np.vstack((X, X_))
        Y = np.vstack((Y, Y_))

    state_dim = Y.shape[1]
    control_dim = X.shape[1] - state_dim

    controller = RbfController(state_dim=state_dim, control_dim=control_dim, num_basis_functions=bf, max_action=max_action)

    R = ExponentialReward(state_dim=state_dim, t=target, W=weights)

    pilco = PILCO(X, Y, controller=controller, horizon=T, reward=R, m_init=m_init, S_init=S_init)

    # for numerical stability
    for model in pilco.mgpr.models:
        # model.kern.lengthscales.prior = gpflow.priors.Gamma(1,10) priors have to be included before
        # model.kern.variance.prior = gpflow.priors.Gamma(1.5,2)    before the model gets compiled
        model.likelihood.variance = 0.001
        model.likelihood.variance.trainable = False

    for rollouts in range(N):
        print("**** ITERATION no", rollouts, " ****")
        pilco.optimize_models(maxiter=maxiter, restarts=2)
        pilco.optimize_policy(maxiter=maxiter, restarts=2)

        X_new, Y_new = rollout(env, pilco, timesteps=T_sim, verbose=True, SUBS=SUBS)

        # Since we had decide on the various parameters of the reward function
        # we might want to verify that it behaves as expected by inspection
        # cur_rew = 0
        # for t in range(0,len(X_new)):
        #     cur_rew += reward_wrapper(R, X_new[t, 0:state_dim, None].transpose(), 0.0001 * np.eye(state_dim))[0]
        # print('On this episode reward was ', cur_rew)

        # Update dataset
        X = np.vstack((X, X_new[:T, :])); Y = np.vstack((Y, Y_new[:T, :]))
        pilco.mgpr.set_XY(X, Y)

        lens.append(len(X_new))
        print(len(X_new))
        if len(X_new) > 120: break
