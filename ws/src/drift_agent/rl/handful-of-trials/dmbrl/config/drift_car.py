from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import numpy as np
import tensorflow as tf
from dotmap import DotMap
import gym

from dmbrl.misc.DotmapUtils import get_required_argument
from dmbrl.modeling.layers import FC
import dmbrl.env


class DriftCarConfigModule:
    ENV_NAME = "DriftCarGazeboContinuousBodyFrame4WD-v1"
    TASK_HORIZON = 150
    NTRAIN_ITERS = 5
    NROLLOUTS_PER_ITER = 10
    PLAN_HOR = 50
    MODEL_IN, MODEL_OUT = 4, 3
    GP_NINDUCING_POINTS = 200

    def __init__(self):
        self.ENV = gym.make(self.ENV_NAME)
        cfg = tf.ConfigProto()
        cfg.gpu_options.allow_growth = True
        self.SESS = tf.Session(config=cfg)
        self.NN_TRAIN_CFG = {"epochs": 5}
        self.OPT_CFG = {
            "Random": {
                "popsize": 2000
            },
            "CEM": {
                "popsize": 200,
                "num_elites": 30,
                "max_iters": 5,
                "alpha": 0.1
            }
        }

    @staticmethod
    def obs_preproc(obs):
        return obs

    @staticmethod
    def obs_postproc(obs, pred):
        return pred

    @staticmethod
    def targ_proc(obs, next_obs):
        return next_obs

    @staticmethod
    def obs_cost_fn(obs):
        desiredAngularVel = -3.5
        desiredForwardVel = 0.5
        desiredSideVel = 2
        target = np.array([desiredAngularVel, desiredForwardVel, desiredSideVel])
        sigma = 5
        if isinstance(obs, np.ndarray):
            return 1 - np.exp(-np.sum(np.square(target-obs))/(2 * sigma**2))
        else:
            return 1 - tf.exp(-tf.reduce_sum(
                tf.square(obs - target), axis=1
            ) / (2 * sigma**2))

    @staticmethod
    def ac_cost_fn(acs):
        if isinstance(acs, np.ndarray):
            return 0
        else:
            return 0

    def nn_constructor(self, model_init_cfg):
        model = get_required_argument(model_init_cfg, "model_class", "Must provide model class")(DotMap(
            name="model", num_networks=get_required_argument(model_init_cfg, "num_nets", "Must provide ensemble size"),
            sess=self.SESS, load_model=model_init_cfg.get("load_model", False),
            model_dir=model_init_cfg.get("model_dir", None)
        ))
        if not model_init_cfg.get("load_model", False):
            model.add(FC(200, input_dim=self.MODEL_IN, activation='swish', weight_decay=0.0001))
            model.add(FC(200, activation='swish', weight_decay=0.00025))
            model.add(FC(200, activation='swish', weight_decay=0.00025))
            model.add(FC(self.MODEL_OUT, weight_decay=0.0005))
        model.finalize(tf.train.AdamOptimizer, {"learning_rate": 0.001})
        return model

    def gp_constructor(self, model_init_cfg):
        model = get_required_argument(model_init_cfg, "model_class", "Must provide model class")(DotMap(
            name="model",
            kernel_class=get_required_argument(model_init_cfg, "kernel_class", "Must provide kernel class"),
            kernel_args=model_init_cfg.get("kernel_args", {}),
            num_inducing_points=get_required_argument(
                model_init_cfg, "num_inducing_points", "Must provide number of inducing points."
            ),
            sess=self.SESS
        ))
        return model



CONFIG_MODULE = DriftCarConfigModule
