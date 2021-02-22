#!/usr/bin/env python

import sys

import numpy as np
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from vswarm.utils import message_util
from scipy.spatial.distance import pdist


class MetricNode:

    def __init__(self):

        rospy.init_node('metric_node', argv=sys.argv)

        self.distance_min_pub = rospy.Publisher('metrics/distance/min',
                                                Float64, queue_size=1)
        self.distance_max_pub = rospy.Publisher('metrics/distance/max',
                                                Float64, queue_size=1)
        self.distance_std_pub = rospy.Publisher('metrics/distance/std',
                                                Float64, queue_size=1)
        self.distance_mean_pub = rospy.Publisher('metrics/distance/mean',
                                                 Float64, queue_size=1)

        # Get all published vision pose topics
        # pose_topic = 'mavros/vision_pose/pose'
        pose_topic = 'global_position/pose'
        pose_msg_type = 'geometry_msgs/PoseStamped'
        pose_topics = {}
        for topic, msg_type in rospy.get_published_topics():
            if pose_topic in topic and msg_type == pose_msg_type:
                namespace = topic.split('/')[1]
                drone_id = int(namespace.split('_')[1]) - 1
                pose_topics[drone_id] = topic

        self.num_agents = len(pose_topics)
        if self.num_agents == 0:
            exit()
        self.positions = {}

        self.pose_subs = []
        for drone_id, topic in pose_topics.iteritems():
            pose_sub = rospy.Subscriber(topic, PoseStamped,
                                        callback=self.pose_callback,
                                        callback_args=drone_id,
                                        queue_size=1)
            self.pose_subs.append(pose_sub)

    def pose_callback(self, pose_msg, drone_id):
        position, orientation = message_util.pose_to_numpy(pose_msg)
        self.positions[drone_id] = position

    def publish_metrics(self):

        # Wait for all positions to be collected
        if len(self.positions) < self.num_agents:
            return

        positions = np.array([pos[:2] for pos in self.positions.itervalues()])
        distances = pdist(positions)
        self.distance_min_pub.publish(Float64(distances.min()))
        self.distance_max_pub.publish(Float64(distances.max()))
        self.distance_mean_pub.publish(Float64(distances.mean()))
        self.distance_std_pub.publish(Float64(distances.std()))

    def run(self):
        self.rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_metrics()
            self.rate.sleep()


def main(args):
    MetricNode().run()


if __name__ == '__main__':
    main(sys.argv)
