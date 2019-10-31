import gym
from gym import error, spaces, utils
from gym.utils import seeding
import rospy
from barc.msg import ECU
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TransformStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf



class DriftCarEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    DEFAULT_STATE_INFO = ["x", "y", "i", "j", "k", "w", "xdot", "ydot", "thetadot", "s", "xdotbodyframe",
                          "ydotbodyframe"]
    def __init__(self, state_info=DEFAULT_STATE_INFO):
        rospy.init_node("drift_car_env")
        self.ecu = rospy.Publisher("ecu_pwm", ECU, queue_size=1)
        self.reward_range = (-np.inf, np.inf)
        self.action_space = spaces.Box(np.array([-0.366519]), np.array([0.366519]))
        self.state_info = state_info
        high = np.ones(len(self.state_info)) * np.finfo(np.float32).max
        self.observation_space = spaces.Box(-high, high)

        self._seed()

        self.radius = 1
        self.throttle = 1750
        self.maxDeviationFromCenter = 4

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        self.publish_throttle_steering(action)

        posData = self.getOdom()
        # imuData = self.getIMUData()


        state = self.getState(posData)
        # rewardState = [state[-4], state[-2], state[-1]]
        reward = self.getReward(state)

        done = self.isDone(posData)

        # self.previous_imu = imuData
        self.previous_pos = posData
        self.previous_action = action
        return state, reward, done, {}


    def _reset(self):
        rospy.sleep(5)
        posData = self.getOdom()
        state = self.getState(posData)
        return state

    def _render(self, mode='human', close=False):
        pass


    def publish_throttle_steering(self, action):
        msg = ECU()
        msg.motor = self.throttle
        msg.servo = action*(500/0.366519) + 1520
        self.ecu.publish(msg)

    def getReward(self, state):
        desiredAngularVel = -3.5
        desiredForwardVel = 0.5
        desiredSideVel = 2
        carAngularVel = state[0]
        carForwardVel = state[1]
        carSideVel = state[2]

        sigma1 = 5
        deviationMagnitude = (carSideVel - desiredSideVel) ** 2 + \
                             (carForwardVel - desiredForwardVel) ** 2 + \
                             (carAngularVel - desiredAngularVel) ** 2
        # 1 - exp for Cost
        # exp - 1 for reward
        return math.exp(-deviationMagnitude / (2 * sigma1 ** 2))


    def isDone(self, posData):
        # Done is true if the car ventures too far from the center of the circular drift
        x = posData.pose.pose.position.x
        y = posData.pose.pose.position.y
        return (self.maxDeviationFromCenter <= ((x ** 2 + y ** 2) ** 0.5))

    def getState(self, posData):
        # odomEstimate = self.getOdomEstimate()
        # velxEstimate = odomEstimate.twist.twist.linear.x
        # velyEstimate = odomEstimate.twist.twist.linear.y

        pose = posData.pose.pose
        position = pose.position
        orientation = pose.orientation

        t = tf.TransformListener()
        m = TransformStamped()
        m.header.frame_id = 'map'
        m.child_frame_id = 'drift_car/base_link'
        m.transform.translation.x = position.x
        m.transform.translation.y = position.y
        m.transform.translation.z = position.z
        m.transform.rotation.x = orientation.x
        m.transform.rotation.y = orientation.y
        m.transform.rotation.z = orientation.z
        m.transform.rotation.w = orientation.w

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        t.setTransform(m)

        velx = posData.twist.twist.linear.x
        vely = posData.twist.twist.linear.y
        velz = posData.twist.twist.linear.z

        # Transform to body frame velocities
        velVector = Vector3Stamped()
        velVector.vector.x = velx
        velVector.vector.y = vely
        velVector.vector.z = velz
        velVector.header.frame_id = "map"
        # velVectorTransformed = self.tl.transformVector3("base_link", velVector)
        t.waitForTransform('/drift_car/base_link', 'map', rospy.Time(), rospy.Duration(0.1))
        velVectorTransformed = t.transformVector3("drift_car/base_link", velVector)
        velxBody = velVectorTransformed.vector.x
        velyBody = velVectorTransformed.vector.y
        velzBody = velVectorTransformed.vector.z

        carTangentialSpeed = math.sqrt(velx ** 2 + vely ** 2)
        carAngularVel = posData.twist.twist.angular.z

        stateInfo = {
            "x": position.x, "y": position.y,
            "i": orientation.x, "j": orientation.y,
            "k": orientation.z, "w": orientation.w,
            "xdot": velx, "ydot": vely,
            "thetadot": carAngularVel,
            "s": carTangentialSpeed,
            "xdotbodyframe": velxBody, "ydotbodyframe": velyBody
        }

        state = []
        for key in self.state_info:
            state.append(stateInfo[key])

            # rewardState = [stateInfo["thetadot"], stateInfo["xdotbodyframe"], stateInfo["ydotbodyframe"]]
        # print(state)
        return np.array(state)

    def getOdom(self):
        # print("Fetching Pos Data")
        failureCount = 0
        odomData = None
        while odomData is None:
            try:
                odomData = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=1)
            except Exception as e:
                failureCount += 1
                print(e)
                pass
        # print("Fetched Pos Data")
        return odomData