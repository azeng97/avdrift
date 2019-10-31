import filter_env
import sys
from ddpg import *
import gc
gc.enable()
import gym
import drift_gym
#ENV_NAME = "DriftCarGazeboContinuous4WD-v0"
ENV_NAME = "DriftCar-v1"
TEST = 10
MAX_STEPS = 100
import argparse
import csv
import numpy as np


def main(args):
    env = filter_env.makeFilteredEnv(gym.make(ENV_NAME))
    agent = DDPG(env)
    # env.monitor.start('experiments/' + ENV_NAME,force=True)
    saver = tf.train.Saver()
    saver.restore(agent.sess, args.checkpoint)
    print("Resuming checkpoint from" + args.checkpoint)
    max_reward = -100000
    rewards = []
    for i in range(TEST):
        state = env.reset()
        for j in range(MAX_STEPS):
            action = agent.action(state) # direct action for test
            state,reward,done,_ = env.step(action)
            rewards.append(reward)
            if done:
                break
        print("Average Reward: {}".format(np.mean(rewards)))



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, type=str)
    args = parser.parse_args(sys.argv[1:])
    main(args)
