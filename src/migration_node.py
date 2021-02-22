#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import argparse
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from vswarm.cfg import MigrationNodeConfig


class MigrationNode:
    """Implements swarm migration."""

    def __init__(self):

        self.node_name = 'migration_node'
        rospy.init_node(self.node_name, argv=sys.argv)

        # Dynamic reconfigure
        self.config = argparse.Namespace()
        self.server = Server(MigrationNodeConfig, callback=self.config_callback)
        self.client = Client(self.node_name)
        self.config = argparse.Namespace(**self.client.get_configuration())

        self.n = rospy.get_param('/gcs/n')
        rospy.loginfo('Got {} agents.'.format(self.n))

        self.poses = rospy.get_param('~poses', default='mavros/vision_pose/pose')
        waypoints_list = rospy.get_param('~waypoints')
        self.waypoints = [np.array([w['x'], w['y'], w['z']]) for w in waypoints_list]
        self.current_waypoint = 0

        migration_topic = 'migration_node/setpoint'
        self.setpoint_pub = rospy.Publisher(migration_topic, PoseStamped, queue_size=1)

        marker_topic = 'migration_node/migration_markers'
        self.waypoint_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=1)

        self.positions = [None] * self.n
        self.pose_subscribers = []
        for i in range(self.n):
            topic = '/drone_{}/{}'.format(i + 1, self.poses)
            pose_sub = rospy.Subscriber(topic, PoseStamped,
                                        callback=self.pose_callback,
                                        callback_args=i)
            self.pose_subscribers.append(pose_sub)

    def config_callback(self, config, level):
        self.config = argparse.Namespace(**config)
        return config

    def pose_callback(self, pose_msg, i):
        rospy.loginfo_once('Got poses')
        p = pose_msg.pose.position
        self.positions[i] = np.array([p.x, p.y, p.z])

    def publish_migration_setpoint(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'
        wp = self.waypoints[self.current_waypoint]
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        pose.pose.position.z = wp[2]
        self.setpoint_pub.publish(pose)

    def publish_migration_markers(self):
        markers = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = i
            marker.pose.position.x = wp[0]
            marker.pose.position.y = wp[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0  # Initialize quaternion!
            marker.color.a = 0.3
            marker.color.r = 0.7
            marker.color.b = 0.7
            marker.color.g = 0.7
            # visualize current waypoint
            if i == self.current_waypoint:
                marker.color.g = 1.0
            marker.scale.x = self.config.acceptance_radius
            marker.scale.y = self.config.acceptance_radius
            marker.scale.z = self.config.acceptance_radius
            markers.markers.append(marker)
        self.waypoint_pub.publish(markers)

    def run(self):

        rate = rospy.Rate(10.)

        while not rospy.is_shutdown():

            rate.sleep()

            if any(x is None for x in self.positions):
                rospy.logwarn_throttle(3, 'Waiting for `{}`.'.format(self.poses))
                continue

            position_mean = np.array(self.positions).mean(axis=0)

            # Check if waypoint is reached
            position_waypoint = self.waypoints[self.current_waypoint]
            position_error = position_waypoint - position_mean
            # TODO: currently ignores z component of waypoints
            distance = np.linalg.norm(position_error[:2])
            if distance < self.config.acceptance_radius:

                rospy.loginfo('Waypoint {}/{} reached.'.format(self.current_waypoint + 1, len(self.waypoints)))

                # Cycle through waypoints (in reversed order if desired)
                increment = -1 if self.config.reversed else 1
                self.current_waypoint += increment
                self.current_waypoint %= len(self.waypoints)

            self.publish_migration_setpoint()
            self.publish_migration_markers()


def main():
    try:
        MigrationNode().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
