#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import sys

import rospy
from vswarm.cfg import RelativeLocalizationNodeConfig
from dynamic_reconfigure.server import Server


class RelativeLocalizationConfigServer:

    def __init__(self):
        rospy.init_node('relative_localization_config_server', argv=sys.argv)
        self.server = Server(RelativeLocalizationNodeConfig,
                             callback=self.config_callback)

    def config_callback(self, config, level):
        rospy.logdebug('Called config_callback in {}.'.format(rospy.get_name()))
        return config


def main():
    RelativeLocalizationConfigServer()
    rospy.spin()


if __name__ == '__main__':
    main()
