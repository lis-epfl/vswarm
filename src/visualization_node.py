#!/usr/bin/env python3

from __future__ import absolute_import, division, print_function

import sys

import rospy
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray, Marker

id_to_color = {
    1: (1.0, 0.0, 0.0),  # Red
    2: (0.0, 1.0, 0.0),  # Green
    3: (0.0, 0.0, 1.0),  # Blue
}


class VisualizationNode:

    def __init__(self):

        self.node_name = 'visualization_node'
        rospy.init_node(self.node_name, argv=sys.argv)

        # Publishers
        topic = f'{self.node_name}/relative_localization'
        self.relative_localization_pub = rospy.Publisher(topic, MarkerArray, queue_size=1)

        topic = f'{self.node_name}/multi_target_tracking'
        self.multi_target_tracking_pub = rospy.Publisher(topic, MarkerArray, queue_size=1)

        # Agents
        self.n = rospy.get_param('~n', default=1)
        self.agents = set(range(1, self.n + 1))

        topic = 'relative_localization_node/detections'
        self.relative_localization_subs = {}
        for agent in self.agents:
            abs_topic = f'/drone_{agent}/{topic}'
            subscriber = rospy.Subscriber(abs_topic, Detection3DArray,
                                          callback=self.relative_localization_callback,
                                          callback_args=agent)
            self.relative_localization_subs[agent] = subscriber

        topic = 'multi_target_tracking_node/detections'
        self.multi_target_tracking_subs = {}
        for agent in self.agents:
            abs_topic = f'/drone_{agent}/{topic}'
            subscriber = rospy.Subscriber(abs_topic, Detection3DArray,
                                          callback=self.multi_target_tracking_callback,
                                          callback_args=agent)
            self.multi_target_tracking_subs[agent] = subscriber

    def multi_target_tracking_callback(self, detection_array_msg, agent):
        rospy.loginfo_once('Got multi-target tracking detections')
        # TODO(fabian): Covariances not correctly rotated (use PoseWithCovariance instead)
        markers = self.get_detection_array_markers(detection_array_msg, agent)
        self.multi_target_tracking_pub.publish(markers)

    def relative_localization_callback(self, detection_array_msg, agent):
        rospy.loginfo_once('Got relative localization detections')
        markers = self.get_detection_array_markers(detection_array_msg, agent)
        self.relative_localization_pub.publish(markers)

    def get_detection_array_markers(self, detection_array_msg, agent):

        markers = MarkerArray()

        # DELETEALL marker is not necessary (the ID is overwritten!)
        # delete_all_marker = Marker()
        # delete_all_marker.id = 0
        # delete_all_marker.action = Marker.DELETEALL
        # markers.markers.append(delete_all_marker)

        marker_id = 1

        for detection in detection_array_msg.detections:

            marker = Marker()
            marker.header = detection.header
            marker.ns = f'drone_{agent}'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = detection.bbox.center.position.x
            marker.pose.position.y = detection.bbox.center.position.y
            marker.pose.position.z = detection.bbox.center.position.z
            marker.pose.orientation.w = 1.0  # Normalize quaternion
            marker.scale.x = detection.bbox.size.x
            marker.scale.y = detection.bbox.size.y
            marker.scale.z = detection.bbox.size.z
            marker.color.a = 0.5
            marker.color.r = id_to_color[agent][0]
            marker.color.g = id_to_color[agent][1]
            marker.color.b = id_to_color[agent][2]

            markers.markers.append(marker)

            marker_id += 1

        return markers


def main():
    VisualizationNode()
    rospy.spin()


if __name__ == '__main__':
    main()
