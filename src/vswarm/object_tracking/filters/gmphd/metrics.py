# -*- coding: utf-8 -*-

from __future__ import absolute_import, division

import numpy as np
from scipy.optimize import linear_sum_assignment


def ospa(X, Y, c=200.0, p=1.0):
    """Optimal subpattern alignment (OSPA) metric.

    Args:
        X (ndarray): Ground truth states as matrix
        Y (ndarray): Estimated states as matrix
        c (float): Cutoff parameter
        p (float): Order parameter
    """

    # Cast to numpy arrays (column vectors)
    X = np.array(X, dtype=float).squeeze().T
    Y = np.array(Y, dtype=float).squeeze().T

    # Ensure X and Y stay matrices even though we only have one dimension!
    if X.ndim == 1:
        X = X[np.newaxis, ...]
    if Y.ndim == 1:
        Y = Y[np.newaxis, ...]

    # If both inputs are empty we have no cost
    if X.size == 0 and Y.size == 0:
        return 0.0

    # If only one of the inputs is zero we incur maximum cost
    if X.size == 0 or Y.size == 0:
        return c

    # Calculate sizes of the input point patterns
    n = X.shape[1]
    m = Y.shape[1]

    # Calculate cost/weight matrix for pairings - fast method with vectorization
    # Original implementation relies on Matlab/Fortran order for reshaping!
    xs = np.tile(X, (1, m))
    ys = np.reshape(np.tile(Y, (n, 1)), (Y.shape[0], n * m), order='F')
    D = np.reshape(np.sqrt(np.sum(np.square(xs - ys), axis=0)), (n, m), order='F')
    D = np.minimum(D, c) ** p

    # Compute optimal assignment and cost using the Hungarian algorithm
    row_ind, col_ind = linear_sum_assignment(D)
    cost = D[row_ind, col_ind].sum()

    # Calculate final distance
    dist = (1. / max(m, n) * (c ** p * abs(m - n) + cost)) ** (1. / p)

    return dist
