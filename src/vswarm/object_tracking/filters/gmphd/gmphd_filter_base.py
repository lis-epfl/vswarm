# -*- coding: utf-8 -*-

"""
This module implements the base class for a GM-PHD filter.
The base GM-PHD filter contains common arguments that are used for any type of filter,
such as the dimensionality of observations and measurements.
The base class also provides functions that are independent of the specific implementation
of GM-PHD filter, e.g. pruning or extracting the filtered states.
"""

from __future__ import absolute_import, division

from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import inv

from .gmphd_component import GmphdComponent

THRESHOLD = 0.5


class GmphdFilterBase(object):
    """GM-PHD filter base class."""

    def __init__(self,
                 dim_x,
                 dim_z,
                 survival_prob,
                 detection_prob,
                 process_cov,
                 observation_cov,
                 clutter_intensity,
                 birth_components):
        """Initializes a GM-PHD filter base instance.

        Arguments:
            dim_x (int): State dimensionality
            dim_z (int): Observation dimensionality
            survival_prob (float): Probability of survival
            detection_prob (float): Probability of detection
            process_cov (ndarray): Process noise covariance
            observation_cov (ndarray): Observation noise covariance
            clutter_intensity (float): Intensity of clutter
            birth_components (list): Birth components

        """
        if dim_x < 1:
            raise ValueError('dim_x must be 1 or greater')
        if dim_z < 1:
            raise ValueError('dim_z must be 1 or greater')

        # Arguments
        self.dim_x = dim_x
        self.dim_z = dim_z

        self.survival_prob = float(survival_prob)
        self.detection_prob = float(detection_prob)
        self.clutter_intensity = float(clutter_intensity)

        self.Q = self.process_cov = process_cov
        self.R = self.observation_cov = observation_cov

        self.components = []
        self.birth_components = deepcopy(birth_components)

    @property
    def components(self):
        return [GmphdComponent(w, m, P) for w, m, P in
                zip(self.w, self.m, self.P)]

    @components.setter
    def components(self, value):
        self.w = np.array([c.weight for c in value])
        self.m = np.array([c.mean for c in value])
        self.P = np.array([c.cov for c in value])

    @property
    def birth_components(self):
        return [GmphdComponent(w, m, P) for w, m, P in
                zip(self.w_birth, self.m_birth, self.P_birth)]

    @birth_components.setter
    def birth_components(self, value):
        self.w_birth = np.array([c.weight for c in value])
        self.m_birth = np.array([c.mean for c in value])
        self.P_birth = np.array([c.cov for c in value])

    def _filter(self, observations, predict_fn, update_fn, control_inputs=None):
        """Update of the GM-PHD filter based on Tab. I from paper.

        Args:
            observations (list): Observation set, each item has a dim_z elements
        """

        # Add birth components
        w_predicts, m_predicts, P_predicts = [], [], []
        if len(self.w_birth) > 0:
            w_predicts.append(self.w_birth)
            m_predicts.append(self.m_birth)
            P_predicts.append(self.P_birth)

        # Prediction
        if len(self.w) > 0:
            m_predict, P_predict = self._predict_multiple(self.m, self.P,
                                                          predict_fn=predict_fn,
                                                          u=control_inputs)
            w_predict = self.survival_prob * self.w
            w_predicts.append(w_predict)
            m_predicts.append(m_predict)
            P_predicts.append(P_predict)

        if len(w_predicts) == 0:
            return

        # Concatenate predictions and birth components
        w_predict = np.concatenate(w_predicts, axis=0)
        m_predict = np.concatenate(m_predicts, axis=0)
        P_predict = np.concatenate(P_predicts, axis=0)

        # Missed detection term
        w_update = (1 - self.detection_prob) * w_predict
        m_update = m_predict
        P_update = P_predict

        zs = np.array(observations)  # N x dim_z

        # Return early if we have no observations
        if len(zs) == 0:
            self.w, self.m, self.P = w_update, m_update, P_update
            return

        # Update
        w_temps, m_temps, P_temps = [], [], []
        qz_temp, m_temp, P_temp = self._update_multiple(zs, m_predict, P_predict,
                                                        update_fn=update_fn)
        for ell in range(len(zs)):
            w_temp = self.detection_prob * w_predict * qz_temp[:, ell]
            w_temp = w_temp / (self.clutter_intensity + np.sum(w_temp))
            w_temps.append(w_temp)
            m_temps.append(m_temp[:, :, ell][..., np.newaxis])
            P_temps.append(P_temp)

        w_temps = np.concatenate(w_temps, axis=0)
        m_temps = np.concatenate(m_temps, axis=0)
        P_temps = np.concatenate(P_temps, axis=0)

        self.w = np.concatenate([w_update, w_temps], axis=0)
        self.m = np.concatenate([m_update, m_temps], axis=0)
        self.P = np.concatenate([P_update, P_temps], axis=0)

    def _predict_multiple(self, ms, Ps, predict_fn, u=None):
        plength = len(ms)
        m_predict = np.zeros_like(ms)
        P_predict = np.zeros_like(Ps)

        for idxp in range(plength):
            m_temp, P_temp = predict_fn(ms[idxp], Ps[idxp], u)
            m_predict[idxp] = m_temp
            P_predict[idxp] = P_temp

        return m_predict, P_predict

    def _update_multiple(self, zs, ms, Ps, update_fn):
        plength = len(ms)
        zlength = len(zs)

        qz_update = np.zeros((plength, zlength))
        m_update = np.zeros((plength, self.dim_x, zlength))
        P_update = np.zeros((plength, self.dim_x, self.dim_x))

        for idxp in range(plength):
            qz_temp, m_temp, P_temp = update_fn(zs.T, ms[idxp], Ps[idxp])
            qz_update[idxp] = qz_temp
            m_update[idxp] = m_temp
            P_update[idxp] = P_temp

        return qz_update, m_update, P_update

    def prune(self, trunc_thresh=1e-6, merge_thresh=0.01, max_components=None):
        """Pruning for the GM-PHD filter based on Tab. II from paper.

        Args:
            trunc_thresh (float): Truncation theshold (T)
            merge_thresh (float): Merging threshold (U)
            max_components (int): Maximum number of Gaussian components (J_max)
        """

        # 1. Truncation using truncation threshold
        self.w, self.m, self.P = self._prune(self.w, self.m, self.P, trunc_thresh)

        # 2. Merging using merge threshold
        self.w, self.m, self.P = self._merge(self.w, self.m, self.P, merge_thresh)

        # 3. CAp only max_components components
        self.w, self.m, self.P = self._cap(self.w, self.m, self.P, max_components)

    def _prune(self, w, m, P, trunc_thresh):
        idx = np.where(w > trunc_thresh)
        return w[idx], m[idx], P[idx]

    def _merge(self, w, m, P, merge_thresh):
        L = len(w)
        I = set(range(L))  # noqa: E741

        w_merged, m_merged, P_merged = [], [], []

        while len(I) > 0:
            j = np.argmax(w)
            Ij = []
            iPt = inv(P[j])
            for i in I:
                x = m[i] - m[j]
                val = x.T.dot(iPt).dot(x)
                if val <= merge_thresh:
                    Ij.append(i)

            ws, ms, Ps = w[Ij], m[Ij], P[Ij]
            w_new = np.sum(ws)
            wstack = np.repeat(ws[..., np.newaxis], self.dim_x, axis=1)[..., np.newaxis]
            m_new = np.sum(wstack * ms, axis=0)
            P_new = np.sum(wstack * Ps, axis=0)

            m_new /= w_new
            P_new /= w_new

            w_merged.append(w_new)
            m_merged.append(m_new)
            P_merged.append(P_new)

            I -= set(Ij)  # noqa: E741 (Set difference!)
            w[Ij] = -1

        return np.array(w_merged), np.array(m_merged), np.array(P_merged)

    def _cap(self, w, m, P, max_components):
        if len(w) > max_components:
            idx = np.argsort(w)[::-1]  # Sort in descending order
            w_new = w[idx[:max_components]]
            w_new *= (np.sum(w) / np.sum(w_new))
            m_new = m[idx[:max_components]]
            P_new = P[idx[:max_components]]
            return w_new, m_new, P_new
        return w, m, P

    def __len__(self):
        return len(self.components)

    def __iter__(self):
        for comp in self.components:
            yield comp

    def __getitem__(self, index):
        return self.components[index]

    def plot_2d(self, ax=None, weight_thresh=0.5):
        """Plot 2-dimensional representation of filter components.

        Arguments:
            ax (matplotlib.axes): Matplotlib axes object
            weight_thresh (float): Weight threshold for plotting

        """
        if ax is None:
            fig, ax = plt.subplots()
        for comp in self.components:
            if comp.weight > weight_thresh:
                comp.plot_2d(ax)

    def extract_states(self, weight_thresh=0.5, bias=1.0):
        """State extraction for GM-PHD filter based on Tab. III from paper

        Args:
            weight_thresh (float): Weight threshold for target extraction
            bias (float): Tendency to prefer false positives over false negatives
        """
        estimates = []
        for comp in self.components:
            weight = comp.weight * float(bias)
            if weight > weight_thresh:
                for _ in range(int(np.round(weight))):
                    estimates.append(comp.mean.copy())
        return estimates

    def extract_states_integral(self, bias=1.0):
        """State extraction based on Dan Stowell's integral implementation.

        Args:
            bias (float): Tendency to perfer false positives over false negatives
        """
        num_estimates = int(round(float(bias) * sum(c.weight for c in self.components)))
        peaks = deepcopy(self.components)
        estimates = []
        while num_estimates > 0:
            index, weight = 0, 0.0
            for i, peak in enumerate(peaks):
                if peak.weight > weight:
                    index = i
                    weight = peak.weight
            estimates.append(peak.mean.copy())
            peaks[index].weight -= 1.0
            num_estimates -= 1
        return estimates

    def __repr__(self):

        # Arguments
        string = 'dim_x = {}\n'.format(self.dim_x)
        string += 'dim_z = {}\n'.format(self.dim_z)
        string += 'survival_prob = {}\n'.format(self.survival_prob)
        string += 'detection_prob = {}\n'.format(self.detection_prob)
        string += 'clutter_intensity = {}\n'.format(self.clutter_intensity)

        # Attributes
        string += 'Q = \n{}\n'.format(self.Q)
        string += 'R = \n{}\n'.format(self.R)

        components = [comp for comp in self.components if comp.weight > THRESHOLD]
        if len(components) > 0:
            string += 'components = [\n'
            for comp in components:
                string += '{},\n'.format(comp)
            string += ']\n'
        else:
            string += 'components = []\n'
        string += ')'

        return string
