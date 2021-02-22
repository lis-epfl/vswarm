# -*- coding: utf-8 -*-

"""
This module implements the unscented GM-PHD filter.
"""

from __future__ import absolute_import, division

from inspect import getsource

import numpy as np
from scipy.linalg import block_diag, cholesky, inv

from .gmphd_filter_base import GmphdFilterBase


class UnscentedGmphdFilter(GmphdFilterBase):
    """Unscented GM-PHD filter"""

    def __init__(self, dt, fx, hx, alpha, beta, kappa, *args, **kwargs):
        """Initializes an unscented GM-PHD filter.

        Arguments:
            dt (float): Time between steps in seconds
            fx (function(x, dt)): State transition function
            hx (function(x)): Observation function
            alpha (float): Spread of sigma points around the mean
            beta (float): Incorporates prior knowledge of the distribution of the mean
            kappa (float): Secondary scaling parameter
        """
        super(UnscentedGmphdFilter, self).__init__(*args, **kwargs)

        self.dt = float(dt)
        self.hx = hx
        self.fx = fx
        self.alpha, self.beta, self.kappa = alpha, beta, kappa

    def filter(self, observations, control_inputs=None):
        self._filter(observations, self._predict, self._update,
                     control_inputs=control_inputs)

    def _predict(self, m, P, u=None):

        # Create block diagonals mu and C
        mu = np.concatenate([m, np.zeros((self.Q.shape[0], 1))])
        C = block_diag(P, self.Q)

        # Generate sigma points
        X_ukf, U = self._unscented_transform(mu, C)

        X_pred = []
        for i in range(len(U)):
            # Partition
            x = X_ukf[:self.dim_x, i][..., np.newaxis]
            v = X_ukf[self.dim_x:, i][..., np.newaxis]
            xp = self.fx(x, u, v)
            X_pred.append(xp)
        X_pred = np.array(X_pred).squeeze().T

        m_predict = X_pred.dot(U[..., np.newaxis])
        X_temp = X_pred - np.repeat(m_predict, len(U), axis=1)
        U[0] += (1 - self.alpha ** 2 + self.beta)
        P_predict = X_temp.dot(np.diag(U)).dot(X_temp.T)

        return m_predict, P_predict

    def _update(self, z, m, P):

        # Create block diagonals mu and C
        mu = np.concatenate([m, np.zeros((self.R.shape[0], 1))])
        C = block_diag(P, self.R)

        # Generate sigma points
        X_ukf, u = self._unscented_transform(mu, C)

        Z_pred = []
        for i in range(len(u)):
            # Partition
            x = X_ukf[:self.dim_x, i][..., np.newaxis]
            e = X_ukf[self.dim_x:, i][..., np.newaxis]
            zp = self.hx(x, e)
            Z_pred.append(zp)
        Z_pred = np.array(Z_pred).squeeze().T

        eta = Z_pred.dot(u)[..., np.newaxis]

        S_temp = Z_pred - np.repeat(eta, len(u), axis=1)
        u[0] += (1 - self.alpha ** 2 + self.beta)
        S = S_temp.dot(np.diag(u)).dot(S_temp.T)

        Vs = cholesky(S)
        det_S = np.prod(np.diagonal(Vs)) ** 2
        inv_sqrt_S = inv(Vs)
        iS = inv_sqrt_S.dot(inv_sqrt_S.T)

        G_temp = X_ukf[:self.dim_x] - np.repeat(m, len(u), axis=1)
        G = G_temp.dot(np.diag(u)).dot(S_temp.T)
        K = G.dot(iS)

        zlength = z.shape[-1]
        etas = np.repeat(eta, zlength, axis=1)
        zs = z - etas

        exponent = np.diagonal(zs.T.dot(iS).dot(zs))
        qz_temp = np.exp(-0.5 * self.dim_z * np.log(2 * np.pi) - 0.5 * np.log(det_S) - 0.5 * exponent)
        m_temp = np.repeat(m, zlength, axis=1) + K.dot(zs)
        P_temp = P - G.dot(iS).dot(G.T)

        return qz_temp, m_temp, P_temp

    def _unscented_transform(self, m, P):
        n_x = len(m)
        lambda_ = self.alpha ** 2 * (n_x + self.kappa) - n_x
        Psqrtm = cholesky((n_x + lambda_) * P).T
        num_sigmas = 2 * n_x + 1

        X = np.repeat(m, num_sigmas, axis=1)
        X += np.concatenate([np.zeros((n_x, 1)), -Psqrtm, Psqrtm], axis=1)

        w = 0.5 * np.ones(num_sigmas)
        w[0] = lambda_
        w /= (n_x + lambda_)

        return X, w

    def __repr__(self):

        # Class name
        string = '{}(\n'.format(self.__class__.__name__)

        # Base class attributes
        string += super().__repr__()

        # Attributes
        string += 'dt = \n{}\n'.format(self.dt)
        string += 'alpha = \n{}\n'.format(self.alpha)
        string += 'beta = \n{}\n'.format(self.beta)
        string += 'kappa = \n{}\n'.format(self.kappa)
        string += 'fx = \n{}\n'.format(getsource(self.fx))
        string += 'hx = \n{}\n'.format(getsource(self.hx))

        return string
