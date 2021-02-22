#!/usr/bin/env python3

"""
This node tracks the positions and velocities of multiple drones based on relative positions.
Input: detections [vision_msgs/Detection3DArray]
Output: detections [vision_msgs/Detection3DArray]
"""

from __future__ import absolute_import, division, print_function

import sys
from copy import deepcopy

import numpy as np
import rospy
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from vswarm.cfg import MultiTargetTrackingNodeConfig
from sklearn.cluster import DBSCAN
from std_msgs.msg import Int32
from vision_msgs.msg import Detection3D, Detection3DArray

from vswarm.object_tracking.utils import cartesian2polar, cartesian2spherical


class MultiTargetTrackingNode:

    def __init__(self):

        self.node_name = 'multi_target_tracking_node'
        rospy.init_node(self.node_name, argv=sys.argv)

        # Try connecting to global dynamic reconfigure server, otherwise create local one
        self.config = {}
        try:
            self.client = Client('/gcs/multi_target_tracking_config_server',
                                 config_callback=self.config_callback,
                                 timeout=1)
            rospy.loginfo('Connected to remote dynamic reconfigure server.')
        except rospy.ROSException:
            rospy.logwarn('Failed to connect to dynamic reconfigure server.')
            rospy.loginfo('Connected to local dynamic reconfigure server.')
            self.server = Server(MultiTargetTrackingNodeConfig, callback=self.config_callback)
            self.client = Client(self.node_name)
        self.config = self.client.get_configuration()

        detections_pub_topic = self.node_name + '/detections'
        self.detections_pub = rospy.Publisher(detections_pub_topic, Detection3DArray,
                                              queue_size=1)

        pose_cov_topic = self.node_name + '/pose_cov'
        self.pose_cov_pub = rospy.Publisher(pose_cov_topic, PoseWithCovarianceStamped,
                                            queue_size=1)

        num_target_topic = self.node_name + '/num_targets'
        self.num_target_pub = rospy.Publisher(num_target_topic, Int32, queue_size=1)

        self.dbscan = DBSCAN(eps=1.0, min_samples=1)
        self.dimensions = 2
        from vswarm.object_tracking.filters import \
            extended_gmphd_filter_2d_polar as gmphd_filter
        self.gmphd = gmphd_filter.get_filter()

        self.velocity = None
        velocity_topic = 'mavros/local_position/velocity_body'
        self.velocity_sub = rospy.Subscriber(velocity_topic, TwistStamped,
                                             callback=self.velocity_callback,
                                             queue_size=1)

        self.last_detections_msg = None
        detection_sub_topic = 'detections'
        self.detections_sub = rospy.Subscriber(detection_sub_topic, Detection3DArray,
                                               callback=self.detections_callback,
                                               queue_size=1)

    def velocity_callback(self, twist_msg):
        rospy.loginfo_once('Got velocity callback')
        v = twist_msg.twist.linear
        self.velocity = np.array([v.x, v.y, v.z])

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

    def filter_dbscan(self, detections_msg):

        # Get frame_id for drone body frame: 'drone_N/camera_array' -> 'drone_N'
        frame_id = detections_msg.header.frame_id.split('/')[0] + '/base_link'

        # Return early if we only have zero or one detection
        if len(detections_msg.detections) <= 1:
            detections_msg.header.frame_id = frame_id
            return detections_msg

        positions = []
        for detection in detections_msg.detections:
            p = detection.bbox.center.position
            position = np.array([p.x, p.y]) if self.dimensions == 2 \
                else np.array([p.x, p.y, p.z])
            positions.append(position)
        positions = np.array(positions)

        # Greedily find all positions with distance less than threshold
        self.dbscan.fit(positions)

        # Merge close positions as the average of all cluster positions
        labels = np.array(self.dbscan.labels_)
        indices = set(labels)
        filtered = np.array([positions[labels == i].mean(axis=0) for i in indices])

        filtered_detections = Detection3DArray()
        filtered_detections.header.frame_id = frame_id
        filtered_detections.header.stamp = detections_msg.header.stamp
        for filtered_detection in filtered:
            detection = Detection3D()
            detection.header = detections_msg.header
            detection.bbox.center.position.x = filtered_detection[0]
            detection.bbox.center.position.y = filtered_detection[1]
            if self.dimensions == 3:
                detection.bbox.center.position.z = filtered_detection[2]
            filtered_detections.detections.append(detection)

        return filtered_detections

    def filter_gmphd(self, detections_msg):

        # Check when the last detection message arrived
        if self.last_detections_msg is None:
            self.last_detections_msg = detections_msg
            rospy.loginfo('GMPHD filter: last detection message initialized')
            return detections_msg

        # Compute time difference (dt) from last message timestamp
        last_detection_time = self.last_detections_msg.header.stamp.to_sec()
        this_detection_time = detections_msg.header.stamp.to_sec()
        dt = this_detection_time - last_detection_time
        self.last_detections_msg = detections_msg

        # Set dt for those GM-PHD filters that have it (e.g. EK-PHD)
        if hasattr(self.gmphd, 'dt'):
            self.gmphd.dt = dt

        # Update process noise covariance matrix (which depends on dt)
        self.gmphd.Q = self._compute_process_noise_covariance(dt)

        # Generate birth components for PHD filter
        birth_components = []
        for detection in detections_msg.detections:
            birth_component = deepcopy(self.gmphd.birth_component)
            p = detection.bbox.center.position
            birth_component.weight = self.config['birth_weight']
            birth_component.mean[0] = p.x
            birth_component.mean[1] = p.y
            if self.dimensions == 3:
                birth_component.mean[2] = p.z
            birth_components.append(birth_component)
        self.gmphd.birth_components = birth_components

        # Convert observations from cartesian into spherical coordinates
        observations = []
        for detection in detections_msg.detections:
            p = detection.bbox.center.position
            observation = cartesian2polar([p.x, p.y]) if self.dimensions == 2 \
                else cartesian2spherical([p.x, p.y, p.z])
            observations.append(observation)
        observations = np.array(observations)

        # Prepare control inputs (estimated velocity)
        control_inputs = np.zeros(self.gmphd.dim_x)
        if self.velocity is not None:
            control_inputs[:self.dimensions] = self.velocity[:self.dimensions]
        control_inputs = control_inputs[..., np.newaxis]

        # Update filter
        self.gmphd.filter(observations, control_inputs)

        # Prune
        self.gmphd.prune(trunc_thresh=self.config['trunc_thresh'],
                         merge_thresh=self.config['merge_thresh'],
                         max_components=self.config['max_components'])

        # Get frame_id for drone body frame: 'drone_N/camera_array' -> 'drone_N'
        frame_id = detections_msg.header.frame_id.split('/')[0] + '/base_link'

        # Publishes the biggest component as PoseWithCovariance (for visualization only!)
        if len(self.gmphd.components) > 0:
            comp = self.gmphd.components[0]
            pose_cov_msg = PoseWithCovarianceStamped()
            pose_cov_msg.header.frame_id = frame_id
            pose_cov_msg.header.stamp = rospy.Time.now()

            # Use mean as position
            pose_cov_msg.pose.pose.position.x = comp.mean[0]
            pose_cov_msg.pose.pose.position.y = comp.mean[1]
            pose_cov_msg.pose.pose.position.z = comp.mean[2]

            # Prepare covariance matrix
            covariance = np.zeros((6, 6))  # 6x6 row-major matrix
            dimz = self.gmphd.dim_z
            covariance[:dimz, :dimz] = comp.cov[:dimz, :dimz]  # Need only position covariance
            pose_cov_msg.pose.covariance = list(covariance.flatten())

            self.pose_cov_pub.publish(pose_cov_msg)

        filtered_detections = Detection3DArray()
        filtered_detections.header.frame_id = frame_id
        filtered_detections.header.stamp = rospy.Time.now()
        for comp in self.gmphd.components:

            # Only report components above weight threshold as detections
            if comp.weight < self.config['weight_thresh']:
                continue

            detection = Detection3D()
            detection.header = detections_msg.header
            detection.bbox.center.position.x = comp.mean[0]
            detection.bbox.center.position.y = comp.mean[1]
            detection.bbox.size.x = 2 * np.sqrt(comp.cov[0, 0])
            detection.bbox.size.y = 2 * np.sqrt(comp.cov[1, 1])
            detection.bbox.size.z = 0.1  # rviz complains about scale 0 otherwise
            if self.dimensions == 3:
                detection.bbox.center.position.z = comp.mean[2]
                detection.bbox.size.z = 2 * np.sqrt(comp.cov[2, 2])

            filtered_detections.detections.append(detection)

        return filtered_detections

    def detections_callback(self, raw_msg):

        use_dbscan, use_gmphd = self.config['use_dbscan'], self.config['use_gmphd']
        filtered_msg = Detection3DArray()
        filtered_msg.header = raw_msg.header

        # Filter detections of same object from multiple cameras using DBSCAN
        # DBSCAN guarantees that we only have one observation per true object
        filtered_msg = self.filter_dbscan(raw_msg) if use_dbscan else raw_msg

        # Filter detections with GM-PHD filter
        # GM-PHD requires at most one observation per true object
        filtered_msg = self.filter_gmphd(filtered_msg) if use_gmphd else filtered_msg

        # Log information
        # num_raw = len(raw_msg.detections)
        num_filtered = len(filtered_msg.detections)
        # rospy.loginfo('Detections raw/filtered: {}/{}'.format(num_raw, num_filtered))

        # Publish everything
        self.detections_pub.publish(filtered_msg)
        self.num_target_pub.publish(Int32(num_filtered))

    def _compute_process_noise_covariance(self, dt):
        """Compute piecewise white noise covariance matrix based on dt
        Source: https://en.wikipedia.org/wiki/Kalman_filter
        """
        I2 = np.eye(self.dimensions)
        Q = np.block([[1. / 4. * dt ** 4 * I2, 1. / 2. * dt ** 3 * I2],
                      [1. / 2. * dt ** 3 * I2, dt ** 2 * I2]])
        Q *= self.config['process_noise'] ** 2
        return Q


def main():
    MultiTargetTrackingNode()
    rospy.spin()


if __name__ == '__main__':
    main()
