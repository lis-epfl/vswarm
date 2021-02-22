#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import argparse
import os
import yaml
import sys

import numpy as np
import cv2 as cv


def make_object_points(size, square):
    object_points = []
    cols, rows = size
    for row in range(rows):
        for col in range(cols):
            object_points.append([row * square, col * square, 0.0])
    return np.array(object_points)


parser = argparse.ArgumentParser()

parser.add_argument('dir', help='calibration directory')
parser.add_argument('-s', '--size', default='8x6', type=str,
                    help='chessboard size')
parser.add_argument('-q', '--square', default=0.108, type=float,
                    help='chessboard square size in meters')
parser.add_argument('-v', '--verbose', action='store_true', default=False,
                    help='verbose output')

args = parser.parse_args()

size = tuple([int(c) for c in args.size.split('x')])
square = args.square
directory = os.path.abspath(args.dir)
verbose = args.verbose

if not os.path.exists(directory):
    print('Directory does not exist: {}'.format(directory), file=sys.stderr)
    exit()

if verbose:
    print('directory: {}'.format(directory))
    print('size: {}'.format(args.size))
    print('square: {} m'.format(args.square))

filenames = []
for filename in os.listdir(directory):
    if filename.endswith('.png'):
        path = os.path.join(directory, filename)
        filenames.append(path)

images = {fn: cv.imread(fn) for fn in filenames}
if verbose:
    print('images: {}'.format(len(images)))

calibpath = os.path.join(directory, 'ost.yaml')
if not os.path.exists(calibpath):
    print('Calibration does not exist: {}'.format(calibpath), file=sys.stderr)
    exit()

calib = yaml.load(open(calibpath, 'r'))

camera_name = calib['camera_name']
image_width = calib['image_width']
image_height = calib['image_height']
image_size = (image_width, image_height)
camera_matrix = np.array(calib['camera_matrix']['data']).reshape((3, 3))
projection_matrix = np.array(calib['projection_matrix']['data']).reshape((3, 4))
distortion_model = np.array(calib['distortion_model'])
distortion_coefficients = np.array(calib['distortion_coefficients']['data'])

if verbose:
    print('camera_name: {}'.format(camera_name))
    print('image_size: {}x{}'.format(image_width, image_height))
    print('image_width: {}'.format(image_width))
    print('image_height: {}'.format(image_height))
    print('camera_matrix: \n{}'.format(camera_matrix))
    print('projection_matrix: \n{}'.format(projection_matrix))
    print('distortion_model: {}'.format(distortion_model))
    print('distortion_coefficients: {}'.format(distortion_coefficients))

if distortion_model != 'equidistant':
    print('Unsupported distortion model: {}'.format(distortion_model))

for path, image in images.iteritems():
    new_camera_matrix = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
        camera_matrix, distortion_coefficients, image_size, R=np.eye(3))
    image_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    image_rect = cv.fisheye.undistortImage(image_gray, camera_matrix,
                                           distortion_coefficients,
                                           Knew=new_camera_matrix)
    flags = (cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE)
    found, corners = cv.findChessboardCorners(image_rect, size, flags=flags)
    if not found and verbose:
        print('No chessboard corners found: {}'.format(path))
        continue

    criteria = (cv.TermCriteria_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    image_points = cv.cornerSubPix(image_rect, corners, (11, 11), (-1, -1), criteria=criteria)
    image_rect_color = cv.cvtColor(image_rect, cv.COLOR_GRAY2BGR)
    image_rect_corners = cv.drawChessboardCorners(image_rect_color, size,
                                                  image_points, found)

    object_points = make_object_points(size, square)
    proj_matrix = projection_matrix[0:3, 0:3]
    print(proj_matrix)
    ok, rvec, tvec = cv.solvePnP(object_points, image_points, proj_matrix,
                                 distortion_coefficients)

    rot, _ = cv.Rodrigues(rvec)
    world_points = np.asmatrix(rot) * np.asmatrix(object_points.squeeze().T) + np.asmatrix(tvec)

    reprojected = new_camera_matrix * world_points  # np.dot(new_camera_matrix, world_points)
    reprojected_points = (reprojected[0:2, :] / reprojected[2, :])

    print(image_points.dtype)
    num, _, dim = image_points.shape
    image_points_reprojected = reprojected_points.T.reshape((num, 1, dim))
    image_points_reprojected = image_points_reprojected.astype(np.float32)
    print(image_points_reprojected.dtype)
    image_rect_corners = cv.drawChessboardCorners(image_rect_corners, size,
                                                  image_points_reprojected,
                                                  found)

    reprojection_errors = image_points.squeeze().T - reprojected_points

    reprojection_error = np.sqrt(np.sum(np.array(reprojection_errors) ** 2) / np.product(reprojection_errors.shape))

    cv.imshow('image_rect', image_rect_corners)
    while True:
        key = cv.waitKey(30)
        if key != -1:  # Press any key
            break
