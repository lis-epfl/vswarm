# -*- coding: utf-8 -*-

"""
This module implements the extended GM-PHD filter.
"""

from __future__ import absolute_import, division

from inspect import getsource

import numpy as np
from scipy.linalg import cholesky, inv

from .gmphd_filter_base import GmphdFilterBase


class ExtendedGmphdFilter(GmphdFilterBase):
    """Extended GM-PHD filter"""

    def __init__(self, dt, fx, fxprime, hx, hxprime, **kwargs):
        """Initializes an extended GM-PHD filter.

        Arguments:
            dt (float): Time between steps in seconds
            fx (function): State transition function
            fxprime (function): Jacobian of state transition function
            hx (function): Observation function
            hxprime (function): Jacobian of observation function
        """
        super(ExtendedGmphdFilter, self).__init__(**kwargs)

        self.dt = float(dt)
        self.fx = fx
        self.fxprime = fxprime
        self.hx = hx
        self.hxprime = hxprime

    def filter(self, observations, control_inputs=None):
        self._filter(observations, self._predict, self._update,
                     control_inputs=control_inputs)

    def _predict(self, m, P, u=None):

        m_predict = self.fx(m, u, dt=self.dt)
        F_ekf, G_ekf = self.fxprime(m)
        P_predict = G_ekf.dot(self.Q).dot(G_ekf.T) + F_ekf.dot(P).dot(F_ekf.T)
        P_predict = (P_predict + P_predict.T) / 2.

        return m_predict, P_predict

    def _update(self, z, m, P):

        eta = self.hx(m)
        H_ekf, U_ekf = self.hxprime(m)
        S = U_ekf.dot(self.R).dot(U_ekf.T) + H_ekf.dot(P).dot(H_ekf.T)
        S = (S + S.T) / 2.
        Vs = cholesky(S)
        det_S = np.prod(np.diagonal(Vs)) ** 2
        inv_sqrt_S = inv(Vs)
        iS = inv_sqrt_S.dot(inv_sqrt_S.T)

        K = P.dot(H_ekf.T).dot(iS)

        zlength = z.shape[-1]
        etas = np.repeat(eta, zlength, axis=1)
        zs = z - etas

        exponent = np.diagonal(zs.T.dot(iS).dot(zs))
        qz_temp = np.exp(-0.5 * self.dim_z * np.log(2 * np.pi) - 0.5 * np.log(det_S) - 0.5 * exponent)
        m_temp = np.repeat(m, zlength, axis=1) + K.dot(zs)
        P_temp = (np.eye(P.shape[0]) - K.dot(H_ekf)).dot(P)

        return qz_temp, m_temp, P_temp

    def __repr__(self):

        # Class name
        string = '{}(\n'.format(self.__class__.__name__)

        # Base class attributes
        string += super().__repr__()

        # Attributes
        string += 'dt = \n{}\n'.format(self.dt)
        string += 'fx = \n{}\n'.format(getsource(self.fx))
        string += 'fxprime = \n{}\n'.format(getsource(self.fxprime))
        string += 'hx = \n{}\n'.format(getsource(self.hx))
        string += 'hxprime = \n{}\n'.format(getsource(self.hxprime))

        return string
