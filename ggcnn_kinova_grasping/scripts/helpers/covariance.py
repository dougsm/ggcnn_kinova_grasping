import numpy as np
from numpy.random import normal as nd

def generate_cartesian_covariance(sd):
    if sd == 0:
        return np.identity(3)
    return np.array([
        [1 + nd(0, sd), nd(0, sd), nd(0, sd)],
        [nd(0, sd), 1 + nd(0, sd), nd(0, sd)],
        [nd(0, sd), nd(0, sd), 1 + nd(0, sd)]
])
