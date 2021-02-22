#!/usr/bin/env python3

"""
This node computes relative 3D positions from 2D object detection bounding boxes.
The bearing is inferred from the camera intrinsic parameters.
The distance is estimated based on the known physical size of the object.

Input: detections [vision_msgs/Detection2DArray]
Output: detections [vision_msgs/Detection3DArray]
"""

from __future__ import absolute_import, division, print_function

import sys

import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray
from vswarm.cfg import RelativeLocalizationNodeConfig

from vswarm.relative_localization import RelativeLocalizer


class RelativeLocalizationNode:

    def __init__(self):

        self.node_name = 'relative_localization_node'
        rospy.init_node(self.node_name, argv=sys.argv)

        # Try connecting to global dynamic reconfigure server, otherwise create local one
        self.config = {}
        try:
            self.client = Client('/gcs/relative_localization_config_server',
                                 config_callback=self.config_callback,
                                 timeout=1)
            rospy.loginfo('Connected to remote dynamic reconfigure server.')
        except rospy.ROSException:
            rospy.logwarn('Failed to connect to dynamic reconfigure server.')
            rospy.loginfo('Connected to local dynamic reconfigure server.')
            self.server = Server(RelativeLocalizationNodeConfig, callback=self.config_callback)
            self.client = Client(self.node_name)
        self.config = self.client.get_configuration()

        # Calibration paramters (one for each camera namespace)
        calibrations_dict = rospy.get_param('~calibration')
        self.localizers = {}
        for camera_ns, calibration_dict in calibrations_dict.items():

            intrinsics = calibration_dict['intrinsics']
            D = calibration_dict['distortion_coeffs']  # Assume equidistant/fisheye

            # Camera matrix from intrinsic parameters
            fx, fy, cx, cy = intrinsics
            K = np.array([[fx, 0., cx],
                          [0., fy, cy],
                          [0., 0., 1.]])

            # Distortion coefficients
            D = np.array(D)

            self.localizers[camera_ns] = RelativeLocalizer(K, D)

        # Transform
        self.buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

        # Publisher
        detections_pub_topic = self.node_name + '/detections'
        self.detections_pub = rospy.Publisher(detections_pub_topic, Detection3DArray,
                                              queue_size=1)

        # Subscriber
        self.detections_sub = rospy.Subscriber('detections', Detection2DArray,
                                               callback=self.detections_callback)

    def config_callback(self, config, level=None):
        for name, new_value in config.items():
            if name == 'groups' or name not in self.config:
                continue
            old_value = self.config[name]
            if old_value != new_value:
                rospy.loginfo('Update `{}`: {} -> {}'.format(name, old_value, new_value))
        return self.update_config(config)

    def update_config(self, config):
        self.config = config
        return config

    def detections_callback(self, detections_msg):
        rospy.loginfo_once('Got detections')

        out_detections_msg = Detection3DArray()
        out_detections_msg.header.stamp = rospy.Time.now()
        out_detections_msg.header.frame_id = detections_msg.header.frame_id

        for detection in detections_msg.detections:

            # Get localizer based on detection frame_id
            frame_id = detection.header.frame_id  # drone_X/camera_X_optical
            camera_ns = frame_id.split('/')[-1].replace('_optical', '')  # camera_X
            localizer = self.localizers[camera_ns]

            # Calculate unit-norm bearing from bounding box and physical object size
            bbox_center = (detection.bbox.center.x, detection.bbox.center.y)
            bbox_size = (detection.bbox.size_x, detection.bbox.size_y)
            object_width = self.config['object_width']
            object_height = self.config['object_height']
            object_depth = self.config['object_depth']
            object_size = (object_width, object_height, object_depth)
            bearing = localizer.detection_to_bearing(bbox_center, bbox_size, object_size)

            # Transform bearing vector from camera/optical frame to body/base_link frame
            source_frame = detection.header.frame_id  # drone_X/camera_X_optical
            target_frame = source_frame.split('/')[0] + '/base_link'

            try:
                transform = self.buffer.lookup_transform(target_frame=target_frame,
                                                         source_frame=source_frame,
                                                         time=rospy.Time(0))
            except Exception as e:
                print(e)
                continue

            point = PointStamped()
            point.header.frame_id = source_frame
            point.header.stamp = rospy.Time.now()
            point.point.x = bearing[0]
            point.point.y = bearing[1]
            point.point.z = bearing[2]

            point_tf = tf2_geometry_msgs.do_transform_point(point, transform)

            out_detection = Detection3D()
            out_detection.header = point_tf.header

            out_detection.bbox.center.position.x = point_tf.point.x
            out_detection.bbox.center.position.y = point_tf.point.y
            out_detection.bbox.center.position.z = point_tf.point.z

            out_detection.bbox.size.x = object_depth
            out_detection.bbox.size.y = object_width
            out_detection.bbox.size.z = object_height

            out_detections_msg.detections.append(out_detection)

        self.detections_pub.publish(out_detections_msg)


def main():
    RelativeLocalizationNode()
    rospy.spin()


if __name__ == '__main__':
    main()
