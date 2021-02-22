#!/usr/bin/env python3

"""
This node controls drones using a flocking algorithm.
Its main inputs are relative positions from visual relative localization.
It provides a fallback using poses obtained from motion capture or GNSS.
Inputs:
    - detections [vision_msgs/Detection3DArray]
    - poses [geometry_msgs/PoseStamped]
Outputs: velocity commands [mavros_msgs/PositionTarget]
"""

from __future__ import absolute_import, division, print_function

import argparse
import re
import sys

import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from vision_msgs.msg import Detection3DArray
from vswarm.cfg import FlockingNodeConfig

from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from vswarm.flocking.reynolds import reynolds
from vswarm.utils import setpoint_util


class FlockingNode:
    """Implements vision-based flocking algorithm."""

    def __init__(self):
        self.node_name = 'flocking_node'
        rospy.init_node(self.node_name, argv=sys.argv)

        param_n = '~n'
        self.n = rospy.get_param(param_n, default=None)
        if self.n is None:
            rospy.logerr('Cannot read parameter: {}'.format(param_n))
            exit()

        # Try connecting to global dynamic reconfigure server, otherwise create local one
        self.config = argparse.Namespace()
        try:
            self.client = Client('/gcs/flocking_node_config_server',
                                 config_callback=self.config_callback,
                                 timeout=1)
            rospy.loginfo('Connected to remote dynamic reconfigure server.')
        except rospy.ROSException:
            rospy.logwarn('Failed to connect to dynamic reconfigure server.')
            rospy.loginfo('Connected to local dynamic reconfigure server.')
            self.server = Server(FlockingNodeConfig, callback=self.config_callback)
            self.client = Client(self.node_name)
        self.config = argparse.Namespace(**self.client.get_configuration())

        self.my_id = int(rospy.get_namespace().split('/')[1].split('_')[-1])
        rospy.loginfo('My ID: {}'.format(self.my_id))

        self.ids = set(range(1, self.n + 1)) - set([self.my_id])  # Set difference!
        rospy.loginfo('Other IDs: {}'.format(list(self.ids)))

        self.last_command = np.zeros(3)
        self.detections = []

        # Transform
        self.buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

        # Publisher
        self.pub_setpoint = rospy.Publisher('mavros/setpoint_raw/local',
                                            PositionTarget,
                                            queue_size=1)

        # Subscribers
        self.pose = None
        self.altitude = None
        topic_pose = 'pose'
        self.sub_pose = rospy.Subscriber(topic_pose, PoseStamped,
                                         callback=self.pose_callback,
                                         queue_size=1)

        topic_detections = 'detections'
        self.sub_detections = rospy.Subscriber(topic_detections, Detection3DArray,
                                               callback=self.detections_callback)

        self.waypoint = None
        topic_migration = 'migration'
        self.sub_migration = rospy.Subscriber(topic_migration, PoseStamped,
                                              callback=self.migration_callback)

        self.poses = {}
        self.subs_poses = {}
        for identity in self.ids:
            namespace = '/drone_{}/'.format(identity)
            topic = re.sub('^/drone_[1-9]/', namespace, self.sub_pose.name)
            sub_pose = rospy.Subscriber(topic, PoseStamped,
                                        callback=self.poses_callback,
                                        callback_args=identity)
            self.subs_poses[identity] = sub_pose

    def migration_callback(self, pose_msg):
        rospy.loginfo_once('Got migration setpoint callback')

        target_frame = f'drone_{self.my_id}/base_link'
        source_frame = 'world'

        try:
            transform = self.buffer.lookup_transform(target_frame=target_frame,
                                                     source_frame=source_frame,
                                                     time=rospy.Time(0))
        except Exception as e:
            print(e)
            return

        point = PointStamped()
        point.header = pose_msg.header
        point.point.x = pose_msg.pose.position.x
        point.point.y = pose_msg.pose.position.y
        point.point.z = pose_msg.pose.position.z

        point_tf = tf2_geometry_msgs.do_transform_point(point, transform)

        self.waypoint = point_tf

    def pose_callback(self, pose_msg):
        """Callback for my own pose"""
        rospy.loginfo_once('Got own pose callback')
        self.pose = pose_msg
        self.altitude = pose_msg.pose.position.z

    def poses_callback(self, pose_msg, identity):
        """Callback for poses of other agents"""
        rospy.loginfo_once('Got other poses callback')
        if self.pose is None:
            return

        target_frame = f'drone_{self.my_id}/base_link'
        source_frame = 'world'

        try:
            transform = self.buffer.lookup_transform(target_frame=target_frame,
                                                     source_frame=source_frame,
                                                     time=rospy.Time(0))
        except Exception as e:
            print(e)
            return

        point = PointStamped()
        point.header = pose_msg.header
        point.point.x = pose_msg.pose.position.x
        point.point.y = pose_msg.pose.position.y
        point.point.z = pose_msg.pose.position.z

        point_tf = tf2_geometry_msgs.do_transform_point(point, transform)
        p = point_tf.point

        self.poses[identity] = np.array([p.x, p.y, p.z])

    def detections_callback(self, detection_array_msg):
        rospy.loginfo_once('Got visual detections callback')
        self.detections = []
        for detection in detection_array_msg.detections:
            pos = detection.bbox.center.position
            self.detections.append(np.array([pos.x, pos.y, pos.z]))

    def config_callback(self, config, level=None):
        old_config = vars(self.config)
        for name, new_value in config.items():
            if name == 'groups' or name not in old_config:
                continue
            old_value = old_config[name]
            if old_value != new_value:
                rospy.loginfo('Update `{}`: {} -> {}'.format(name, old_value, new_value))
        return self.handle_config(config)

    def handle_config(self, config):
        self.config = argparse.Namespace(**config)
        return config

    def flock(self):

        # Command from reynolds
        command_reynolds = self.get_command_reynolds()

        # Low-pass filter
        alpha = self.config.smoothing_factor
        command = alpha * command_reynolds + (1 - alpha) * self.last_command
        self.last_command = command

        # For experiments
        command *= self.config.command_gain

        # Altitude controller
        if self.config.use_altitude and self.altitude is not None:
            altitude_error = self.config.altitude_setpoint - self.altitude
            altitude_command = altitude_error * self.config.altitude_gain
            command[2] = altitude_command

        # Add migration term to command
        command += self.get_command_migration()

        # Limit maximum speed to max_speed
        if np.linalg.norm(command) > self.config.max_speed:
            command /= np.linalg.norm(command)  # Make unit vector
            command *= self.config.max_speed  # Scale by max speed

        # Publish velocity setpoint in ENU body frame
        frame_id = f'/drone_{self.my_id}/base_link'
        setpoint_msg = setpoint_util.velocity_target(command, frame_id=frame_id)
        setpoint_msg.coordinate_frame = setpoint_msg.FRAME_BODY_NED
        setpoint_msg.yaw = np.deg2rad(0.)  # Setting to zero not necessary for stability!
        self.pub_setpoint.publish(setpoint_msg)

    def get_command_migration(self):

        # Return early with zero command if possible
        migration_gain = self.config.migration_gain
        if migration_gain <= 0.0 or self.pose is None or self.waypoint is None:
            return np.zeros(3)

        # Return zero command if the last waypoint message is older than one second
        if rospy.Time.now() - self.waypoint.header.stamp > rospy.Duration(1):
            rospy.logwarn('No longer receiving new waypoints. Stopping migration.')
            self.waypoint = None
            return np.zeros(3)

        p = self.waypoint.point
        pos_waypoint = np.array([p.x, p.y, 0])  # Set z component to zero!

        migration_direction = pos_waypoint / np.linalg.norm(pos_waypoint)

        command_migration = migration_direction * migration_gain

        return command_migration

    def get_command_reynolds(self):

        # Decide if flocking based on true poses or visual detections
        if self.config.use_vision:
            positions_rel = self.detections
        else:
            positions_rel = list(self.poses.values())

        # Only flock when we have relative positions
        if len(positions_rel) == 0:
            return np.zeros(3)

        distances = [np.linalg.norm(r) for r in positions_rel]
        agents_outside = [d for d in distances if d > self.config.perception_radius]
        if len(agents_outside):
            msg = 'Agents outside of perception radius: {}'.format(len(agents_outside))
            rospy.logwarn_throttle(1.0, msg)

        command_reynolds = reynolds(positions_rel,
                                    separation_gain=self.config.separation_gain,
                                    cohesion_gain=self.config.cohesion_gain,
                                    alignment_gain=self.config.alignment_gain,
                                    perception_radius=self.config.perception_radius,
                                    max_agents=self.config.max_agents)

        return command_reynolds


def main():
    flocking_node = FlockingNode()
    rate = rospy.Rate(10.)
    try:
        while not rospy.is_shutdown():
            flocking_node.flock()
            rate.sleep()
    except rospy.ROSInterruptException:
        exit()


if __name__ == '__main__':
    main()
