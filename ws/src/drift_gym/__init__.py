import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='DriftCarGazeboPartialBodyFrame4WD-v1',
    entry_point='drift_gym.envs:GazeboEnv',
    # state: thetadot, xDotBody, yDotBody.
    kwargs={'four_wheel_drive': True}
)

register(
    id='DriftCarGazeboContinuousBodyFrame4WD-v1',
    entry_point='drift_gym.envs:GazeboEnv',
    # state: thetaDot, xDotBodyFrame, yDotBodyFrame
    kwargs={'continuous' : True, 'four_wheel_drive': True, 'state_info': ["thetadot", "xdotbodyframe", "ydotbodyframe"]}
)

register(
    id='DriftCarGazeboContinuous4WD-v1',
    entry_point='drift_gym.envs:GazeboEnv',
    # state: thetaDot, xDotBodyFrame, yDotBodyFrame
    kwargs={'continuous' : True, 'four_wheel_drive': True}
)

register(
    id='DriftCar-v1',
    entry_point='drift_gym.envs:DriftCarEnv'
)