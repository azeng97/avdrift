#!/usr/bin/env python
import rospy
import gym
import drift_gym
from sensor_msgs.msg import Joy
import subprocess
from std_msgs.msg import Float64MultiArray
import numpy as np

MIN_THROTTLE = 1500
DRIFT_THROTTLE = 1770
MAX_THROTTLE = 2000
MAX_SERVO = 0.80 #0.366519
env = None

global drifting

drifting = False

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def callback(data):
    if data.buttons[1] == 1: #B Button for Reset
        env.reset()
        return
        
    servo = data.axes[0] * MAX_SERVO
    throtle = translate(data.axes[5], -1.0, 1.0, MAX_THROTTLE, MIN_THROTTLE)

    global drifting
    if data.buttons[4] == 1: # LB Button to stop drift throttle
        drifting = False
    if data.buttons[5] == 1: # RB Button to start drift throttle
        drifting = True
    
    if drifting:
        # print((MAX_THROTTLE, servo))
        state, _, _, _ = env.step((DRIFT_THROTTLE, servo))
        print(state)
        stateArray = Float64MultiArray()
        state = state.tolist()
        state.append(DRIFT_THROTTLE)
        state.append(servo)
        stateArray.data = state
        pub.publish(stateArray)
    else:
        print("Throttle: " + str(throtle))
        print("Servo: " + str(servo))
        state, _, _, _ = env.step((throtle, servo))
        print(state)


def callback2(data, args):
    servo = data.data[0]
    throttle = data.data[1]
    rospy.loginfo(rospy.get_caller_id() + ' Action: %s', data.data)

    env = args[0]
    pub = args[1]
    allRewards = args[2]

    if (servo == -1000):
        calc(allRewards)
        allRewards = []
        rospy.loginfo('Resetting Env . . . \n\n')
        env.reset()
        return

    state, reward, done, _ = env.step((throttle, servo))
    stateArray = Float64MultiArray()
    state = state.tolist()
    state.append(throttle)
    state.append(servo)
    stateArray.data = state
    pub.publish(stateArray)
    allRewards.append(reward)

def calc(rewards):
    r = np.array(rewards)
    print("Mean: ")
    print(np.mean(r))
    print("Std dev: ")
    print(np.std(r))

if __name__=="__main__":
    env = gym.make('DriftCarGazeboContinuousBodyFrame4WD-v1')
    env.reset()
    env.render()

    subprocess.Popen(["rosrun", "joy", "joy_node"])
    rospy.Subscriber('/joy', Joy, callback, queue_size=1)
    global pub
    pub = rospy.Publisher('drift_car/state', Float64MultiArray, queue_size=1)

    allRewards = []
    rospy.Subscriber('drift_car/action', Float64MultiArray, callback2, (env, pub, allRewards))

    print("========================================")
    print("Joystick Controller for Gazebo Drift Car")
    print("========================================\n")
    
    print("LT for acceleration")
    print("RB/LB to toggle constant speed")
    print("B for hard reset")

    rospy.spin()

    env.close()