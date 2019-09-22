# prob_mbrl
Implementation of Deep-PILCO and variants for probabilistic Model Based RL. This is an (in progress) re-implementation of the algorithms in https://github.com/mcgillmrl/kusanagi. We also aim to add other probabilistic model-based RL methods to this library.

## Recommended way to install:

Install the Miniconda 3 distribution: https://conda.io/miniconda.html

    conda install pytorch cuda90 cudnn -c pytorch
    conda install tqdm
 
 To run the mc-pilco cartpole examples, you'll need to also install the kusanagi library (https://github.com/mcgillmrl/kusanagi). We plan to remove this dependency in the future.
 
 For example on how to use this library, take a look at the notbooks folder. currently, we have an example for using the BNN models for regression, and an example of MC PILCO



