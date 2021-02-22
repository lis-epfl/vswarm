# -*- coding: utf-8 -*-

"""
This module implements the linear GM-PHD filter.
"""

from __future__ import absolute_import, division

import numpy as np
from scipy.linalg import cholesky

from .gmphd_filter_base import GmphdFilterBase


class GmphdFilter(GmphdFilterBase):
    """GM-PHD filter."""

    def __init__(self, state_transition_matrix, observation_matrix, **kwargs):
        """Initializes a linear GM-PHD filter.

        Arguments:
            state_transition_matrix (ndarray): State transition matrix
            observation_matrix (ndarray): Observation matrix
        """
        super(GmphdFilter, self).__init__(**kwargs)

        self.F = self.state_transition_matrix = state_transition_matrix
        self.H = self.observation_matrix = observation_matrix

    def filter(self, observations, control_inputs=None):
        self._filter(observations, self._predict, self._update,
                     control_inputs=control_inputs)

    def _predict(self, m, P, u=None):

        m_predict = self.F.dot(m) + u  # TODO: should be self.B.dot(u)
        P_predict = self.Q + self.F.dot(P).dot(self.F.T)

        return m_predict, P_predict

    def _update(self, z, m, P):

        mu = self.H.dot(m)
        S = self.R + self.H.dot(P).dot(self.H.T)
        Vs = cholesky(S)
        det_S = np.prod(np.diagonal(Vs)) ** 2
        inv_sqrt_S = np.linalg.inv(Vs)
        iS = inv_sqrt_S.dot(inv_sqrt_S.T)
        K = P.dot(self.H.T).dot(iS)

        zlength = z.shape[-1]
        etas = np.repeat(mu, zlength, axis=1)
        zs = z - etas

        exponent = np.diagonal(zs.T.dot(iS).dot(zs))
        qz_temp = np.exp(-0.5 * self.dim_z * np.log(2 * np.pi) - 0.5 * np.log(det_S) - 0.5 * exponent)
        m_temp = np.repeat(m, zlength, axis=1) + K.dot(zs)
        P_temp = (np.eye(P.shape[0]) - K.dot(self.H)).dot(P)

        return qz_temp, m_temp, P_temp

    def __repr__(self):

        # Class name
        string = '{}(\n'.format(self.__class__.__name__)

        # Base class attributes
        string += super().__repr__()

        # Attributes
        string += 'F = \n{}\n'.format(self.F)
        string += 'H = \n{}\n'.format(self.H)

        return string
