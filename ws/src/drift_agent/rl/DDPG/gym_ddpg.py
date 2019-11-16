import filter_env
import sys
from ddpg import *
import gc
gc.enable()
import gym
import drift_gym
ENV_NAME = "DriftCarGazeboContinuousBodyFrame4WD-v1"
# ENV_NAME = "DriftCar-v1"
EPISODES = 100000
TEST = 10
MAX_STEPS = 300
import argparse
import csv

def main(args):
    env = filter_env.makeFilteredEnv(gym.make(ENV_NAME))
    agent = DDPG(env)
    # env.monitor.start('experiments/' + ENV_NAME,force=True)
    saver = tf.train.Saver()
    if args.checkpoint:
        saver.restore(agent.sess, args.checkpoint)
        print("Resuming checkpoint from" + args.checkpoint)
    max_reward = -100000
    for episode in range(EPISODES):
        state = env.reset()
        print("episode:", episode)
        # Train
        for step in range(MAX_STEPS):
            action = agent.noise_action(state)
            next_state,reward,done,_ = env.step(action)
            agent.perceive(state,action,reward,next_state,done)
            state = next_state
            if done:
                break
        # Testing:
        if episode % 100 == 0 and episode > 100:
            total_reward = 0
            for i in range(TEST):
                state = env.reset()
                for j in range(MAX_STEPS):
                    #env.render()
                    action = agent.action(state) # direct action for test
                    state,reward,done,_ = env.step(action)
                    total_reward += reward
                    if done:
                        break
            ave_reward = total_reward/TEST
            if ave_reward > max_reward:
                max_reward = ave_reward
                saver.save(agent.sess, "models/ddpg_ep" + str(episode) + "-" + str(ave_reward))
            print('episode: ',episode,'Evaluation Average Reward:',ave_reward)
            with open("models/ddpg_2.csv", "a") as savefile:
                wr = csv.writer(savefile, dialect="excel")
                wr.writerow([episode, ave_reward])

    env.monitor.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=str)
    args = parser.parse_args(sys.argv[1:])
    main(args)
