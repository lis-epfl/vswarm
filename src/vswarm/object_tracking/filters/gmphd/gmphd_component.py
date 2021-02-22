# -*- coding: utf-8 -*-

"""
This module implements a GM-PHD filter component which is a essentially a multivariate
Gaussian parametrized by a mean and covariance, as well as an associated weight.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal


class GmphdComponent(object):
    """Single Gaussian component of a GM-PHD filter."""

    def __init__(self, weight, mean, cov):
        """Initializes a GM-PHD filter component.

        Args:
            weight (float): Weight of the component
            mean (ndarray): Mean vector of the component
            cov (ndarray): Covariance matrix of the component

        """
        self.weight = float(weight)
        self.mean = np.array(mean, dtype=float)
        self.cov = np.array(cov, dtype=float)
        self.mean = np.reshape(self.mean, (self.mean.size, 1))  # Enforce column vector

    def __repr__(self):
        string = '{}(\n'.format(self.__class__.__name__)
        string += 'weight={}\n'.format(self.weight)
        string += 'mean=\n{}\n'.format(self.mean.squeeze())
        string += 'cov=\n{}\n'.format(self.cov)
        string += ')'
        return string

    def __len__(self):
        return len(self.mean)

    def plot_2d(self, ax=None):
        fig = None
        if ax is None:
            fig, ax = plt.subplots()
        mean = self.mean[:2].squeeze()
        cov = self.cov[:2, :2]
        stds = 3
        xs = np.linspace(mean[0] - stds * np.sqrt(cov[0, 0]),
                         mean[0] + stds * np.sqrt(cov[0, 0]))
        ys = np.linspace(mean[1] - stds * np.sqrt(cov[1, 1]),
                         mean[1] + stds * np.sqrt(cov[1, 1]))
        X, Y = np.meshgrid(xs, ys)
        zs = np.array([multivariate_normal.pdf(np.array([x, y]), mean, cov)
                       for x, y in zip(X.ravel(), Y.ravel())])
        Z = zs.reshape(X.shape)
        ax.set_aspect('equal')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        cs = ax.contour(X, Y, Z)
        if fig is not None:
            fig.colorbar(cs)
