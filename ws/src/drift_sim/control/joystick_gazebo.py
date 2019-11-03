#!/usr/bin/env python
import rospy
import gym
import drift_gym
from sensor_msgs.msg import Joy
import subprocess

MIN_THROTTLE = 1500
DRIFT_THROTTLE = 1780
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
        print(state[-4:])
    else:
        print("Throttle: " + str(throtle))
        print("Servo: " + str(servo))
        state, _, _, _ = env.step((throtle, servo))
        print(state[-4:])


if __name__=="__main__":
    env = gym.make('DriftCarGazeboContinuous4WD-v1')
    env.reset()
    env.render()

    subprocess.Popen(["rosrun", "joy", "joy_node"])
    rospy.Subscriber('/joy', Joy, callback, queue_size=1)

    print("========================================")
    print("Joystick Controller for Gazebo Drift Car")
    print("========================================\n")
    
    print("LT for acceleration")
    print("RB/LB to toggle constant speed")
    print("B for hard reset")

    rospy.spin()

    env.close()