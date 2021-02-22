"""
This module performs visual relative localization.
It takes pixel coordinates as inputs and computes the bearing to the real-world location.
"""

import cv2 as cv
import numpy as np


class RelativeLocalizer:
    """A relative localization wrapper that precomputes the inverse camera matrix"""

    def __init__(self, K, D):
        self.K = K
        self.D = D
        self.K_inv = np.linalg.inv(K)  # precompute for efficiency!

    def detection_to_bearing(self, bbox_center, bbox_size, object_size):
        K, D, K_inv = self.K, self.D, self.K_inv
        return detection_to_bearing(bbox_center, bbox_size, object_size, K, D, K_inv)

    def point_to_bearing(self, bbox_center):
        K, D, K_inv = self.K, self.D, self.K_inv
        return point_to_bearing(bbox_center, K, D, K_inv)


def detection_to_bearing(bbox_center, bbox_size, object_size, K, D, K_inv):
    """Convert a 2D bounding box to a 3D unit-norm bearing vector based on object size

    Args:
        bbox_center (tuple): Bounding box center in pixel coordinates (x, y)
        bbox_size (tuple): Bounding box size in pixel coordinates (width, height)
        object_size (tuple): Physical size of the object (width, height, depth)
        K (ndarray): Camera matrix with focal lengths and optical center (fx, fy, cx, cy)
        D (ndarray): Fisheye distortion parameters (k1, k2, k3, k4)
        K (ndarray): Inverse camera matrix

    Returns:
        bearing (ndarray): Unit-norm 3D bearing vector to point (x, y, z)
    """

    x, y = bbox_center
    width_image, _ = bbox_size
    width_object, _, _ = object_size

    # Create point to the bbox center and right edge
    point_center = np.array([x, y])
    point_right = np.array([x + width_image / 2., y])

    # Calculate the bearing to the center of the object
    bearing_center = point_to_bearing(point_center, K, D, K_inv)

    # Calculate bearing to the right edge of the object
    bearing_right = point_to_bearing(point_right, K, D, K_inv)

    # Calculate the angle between the two bearing vectors (already normalized!)
    angle = np.arccos(bearing_center.dot(bearing_right))

    # Calculate distance to object with known object width (and optionally depth)
    distance = ((width_object / 2.) / np.tan(angle))  # + (self.object_depth / 2)

    # Scale bearing vector by distance
    bearing = bearing_center * distance

    return bearing


def point_to_bearing(bbox_center, K, D, K_inv):
    """Convert a 2D point in pixel coordinates to a unit-norm 3D bearing vector.

    Args:
        bbox_center (tuple): Bounding box center in pixel coordinates (x, y)
        K (ndarray): Camera matrix with focal lengths and optical center (fx, fy, cx, cy)
        D (ndarray): Fisheye distortion parameters (k1, k2, k3, k4)
        K (ndarray): Inverse camera matrix

    Returns:
        bearing (ndarray): Unit-norm 3D bearing vector to point (x, y, z)
    """

    # Undistort points from fisheye as if they were created by a pinhole camera
    image_point_cv = bbox_center.reshape(1, -1, 2)
    undistorted_image_point_cv = cv.fisheye.undistortPoints(image_point_cv, K, D, P=K)
    undistorted_image_point = undistorted_image_point_cv.reshape(2)

    # Unproject points using inverse camera matrix with homogeneous coordinates
    undistorted_image_point_homogeneous = np.array([*undistorted_image_point, 1.0])
    world_point = K_inv.dot(undistorted_image_point_homogeneous)
    bearing_norm = world_point / np.linalg.norm(world_point)
    return bearing_norm
