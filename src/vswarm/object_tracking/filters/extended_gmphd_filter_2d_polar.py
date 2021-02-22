from __future__ import absolute_import, division, print_function

from .gmphd import ExtendedGmphdFilter, GmphdComponent
import argparse
import numpy as np

from ..utils import cartesian2polar


def get_filter():

    args = argparse.Namespace()

    args.dt = 0.1  # [s]
    args.dim_state = 4  # [px, py, vx, vy] [m]
    args.dim_obs = 2  # [range, bearing] [m, rad]
    args.std_process = 1.0  # [m/s^2]
    args.std_range = 0.2  # [m]
    args.std_bearing = np.deg2rad(3.)  # [rad]
    args.num_clutter = 0
    args.max_components = 100
    args.prob_detection = 1.0  # [%]
    args.prob_survival = 0.99  # [%]
    args.thresh_trunc = 1e-5
    args.thresh_merge = 0.5
    args.birth_weight = 1e-6  # [%]
    args.std_pos = 1.0  # [m]
    args.std_vel = 1.0  # [m/s]

    clutter_intensity = args.num_clutter / 10 ** 2

    birth_mean = np.zeros(args.dim_state)
    birth_cov = np.diag([args.std_pos, args.std_pos, args.std_vel, args.std_vel]) ** 2

    F = np.eye(args.dim_state)
    F[:args.dim_obs, args.dim_obs:] = np.eye(args.dim_obs) * args.dt

    Q = get_Q(args.dt, args.dim_obs, args.std_process)

    R = np.diag([args.std_range, args.std_bearing]) ** 2

    def fx(x, u=None, v=None, dt=args.dt):
        y = F.dot(x)
        if u is not None:
            y += (u * dt)
        if v is not None:
            y += v
        return y

    def hx(x, e=None):
        x = np.squeeze(x)
        px, py = x[0], x[1]
        r, azimuth = cartesian2polar([px, py])
        y = np.array([r, azimuth])[..., np.newaxis]
        if e is not None:
            y += e
        return y

    def hxprime(x):
        x = np.squeeze(x)
        px, py = x[0], x[1]
        denom = px ** 2 + py ** 2
        H = np.array([[px / np.sqrt(denom), py / np.sqrt(denom), 0, 0],
                      [-py / denom, px / denom, 0, 0]])
        return H, np.eye(args.dim_obs)

    def fxprime(x):
        return F, np.eye(args.dim_state)

    birth_component = GmphdComponent(args.birth_weight, birth_mean, birth_cov)

    gmphd = ExtendedGmphdFilter(dim_x=args.dim_state,
                                dim_z=args.dim_obs,
                                dt=args.dt,
                                fx=fx,
                                fxprime=fxprime,
                                hx=hx,
                                hxprime=hxprime,
                                survival_prob=args.prob_survival,
                                detection_prob=args.prob_detection,
                                process_cov=Q,
                                observation_cov=R,
                                birth_components=[],
                                clutter_intensity=clutter_intensity)

    gmphd.birth_component = birth_component

    return gmphd


def get_Q(dt, dim_obs, std_process):
    I2 = np.eye(dim_obs)
    Q = np.block([[1 / 4 * dt ** 4 * I2, 1 / 2 * dt ** 3 * I2],
                  [1 / 2 * dt ** 3 * I2, dt ** 2 * I2]])
    Q *= std_process ** 2
    return Q


def get_birth_component():
    pass
