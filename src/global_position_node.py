#!/usr/bin/env python3

import sys

import numpy as np
import rospy
import utm
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class GlobalPositionNode:
    """Implements global position as an offset from home position."""

    def __init__(self):
        self.node_name = 'global_position_node'
        rospy.init_node(self.node_name, argv=sys.argv)

        origin_latitude = rospy.get_param('~origin_latitude')
        origin_longitude = rospy.get_param('~origin_longitude')
        origin_altitude = rospy.get_param('~origin_altitude')
        origin_altitude_offset = rospy.get_param('~origin_altitude_offset')

        east, north, _, _ = utm.from_latlon(origin_latitude, origin_longitude)
        up = origin_altitude + origin_altitude_offset

        self.origin = np.array([east, north, up])
        self.orientation = None
        self.east = None
        self.north = None
        self.up = None

        self.position_pub = rospy.Publisher('global_position/pose', PoseStamped,
                                            queue_size=1)

        self.global_position_sub = rospy.Subscriber('mavros/global_position/global',
                                                    NavSatFix,
                                                    callback=self.global_position_callback,
                                                    queue_size=1)
        self.orientation_sub = rospy.Subscriber('mavros/global_position/local', Odometry,
                                                callback=self.orientation_callback,
                                                queue_size=1)

    def global_position_callback(self, nav_msg):
        rospy.loginfo_once('Got global position message')
        self.east, self.north, _, _ = utm.from_latlon(nav_msg.latitude, nav_msg.longitude)
        self.up = nav_msg.altitude

    def orientation_callback(self, odom_msg):
        rospy.loginfo_once('Got orientation')
        self.orientation = odom_msg.pose.pose.orientation

    def publish_position(self):

        if self.east is None or self.north is None or self.up is None or self.orientation is None:
            rospy.logwarn_throttle(3, 'Waiting for `mavros/global_position/{global,local}`.')
            return

        # Compute global position as offset from origin
        position = np.array([self.east, self.north, self.up])
        global_position = position - self.origin

        # Construct global pose from MAVROS messages
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = global_position[0]
        pose.pose.position.y = global_position[1]
        pose.pose.position.z = global_position[2]
        pose.pose.orientation = self.orientation
        self.position_pub.publish(pose)


def main():
    global_position_node = GlobalPositionNode()
    rate = rospy.Rate(10.)
    try:
        while not rospy.is_shutdown():
            global_position_node.publish_position()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
