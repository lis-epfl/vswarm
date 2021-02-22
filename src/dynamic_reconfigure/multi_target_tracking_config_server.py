#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import sys

import rospy
from vswarm.cfg import MultiTargetTrackingNodeConfig
from dynamic_reconfigure.server import Server


class MultiTargetTrackingConfigServer:

    def __init__(self):
        rospy.init_node('multi_target_tracking_config_server', argv=sys.argv)
        self.server = Server(MultiTargetTrackingNodeConfig, callback=self.config_callback)

    def config_callback(self, config, level):
        rospy.loginfo('Updating dynamic reconfigure config.')
        return config


def main():
    MultiTargetTrackingConfigServer()
    rospy.spin()


if __name__ == '__main__':
    main()
