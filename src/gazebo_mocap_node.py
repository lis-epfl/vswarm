#!/usr/bin/env python3
"""Relays Gazebo model states as MAVROS vision pose messages."""
from __future__ import absolute_import, division, print_function

import sys

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped


class GazeboMocapNode:

    def __init__(self):

        # Initialize node
        rospy.init_node('gazebo_mocap_node', argv=sys.argv)

        self.num_agents = rospy.get_param('~num_agents')
        self.vehicle = 'drone'

        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Vision pose publishers
        self.vision_pose_pubs = []
        for i in range(self.num_agents):
            topic = '/{}_{}/mavros/vision_pose/pose'.format(self.vehicle, i + 1)
            vision_pose_pub = rospy.Publisher(topic, PoseStamped, queue_size=1)
            self.vision_pose_pubs.append(vision_pose_pub)

        # Model states subscriber
        self.model_states_msg = None
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states',
                                                 ModelStates,
                                                 callback=self.model_states_callback,
                                                 queue_size=1)

    def send_transform(self):

        # Wait to receive model states message before publishing
        if self.model_states_msg is None:
            return

        # Convert Gazebo model states into pose and twist
        for i, name in enumerate(self.model_states_msg.name):
            if name.startswith(self.vehicle):
                index = int(name.split('_')[-1]) - 1
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'world'
                pose.pose = self.model_states_msg.pose[i]
                self.vision_pose_pubs[index].publish(pose)
                child_frame = '{}_{}'.format(self.vehicle, index + 1)

                transform = TransformStamped()

                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = 'world'
                transform.child_frame_id = child_frame

                p = pose.pose

                transform.transform.translation.x = p.position.x
                transform.transform.translation.y = p.position.y
                transform.transform.translation.z = p.position.z

                transform.transform.rotation.x = p.orientation.x
                transform.transform.rotation.y = p.orientation.y
                transform.transform.rotation.z = p.orientation.z
                transform.transform.rotation.w = p.orientation.w
                self.broadcaster.sendTransform(transform)

    def model_states_callback(self, model_states_msg):
        self.model_states_msg = model_states_msg

    def run(self):
        rate_param = rospy.get_param('~rate', default=100)
        rate = rospy.Rate(rate_param)
        while not rospy.is_shutdown():
            self.send_transform()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


def main():
    GazeboMocapNode().run()


if __name__ == '__main__':
    main()
