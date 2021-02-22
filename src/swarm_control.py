#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import argparse
import sys

import numpy as np
import rospy
from mavros_msgs.msg import ExtendedState, HomePosition
from mavros_msgs.srv import (CommandBool, CommandBoolRequest, CommandTOL,
                             CommandTOLRequest, SetMode, SetModeRequest)
from rosgraph import masterapi


def get_drone(topic):
    return topic.split('/')[1]


class SwarmControl:

    def __init__(self):

        rospy.init_node('swarm_control', argv=sys.argv)

        self.timeout = 1.0  # Timeout for services

    def run(self):

        parser = argparse.ArgumentParser(
            description='Swarm control',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter
        )

        commands = ['arm', 'disarm', 'land', 'offboard', 'takeoff']
        parser.add_argument('command', choices=commands, help='Command')
        parser.add_argument('agent', type=int, nargs='?', default=None,
                            help='Agent ID')
        args = parser.parse_args()

        master = masterapi.Master('/')
        system_state = master.getSystemState()
        self.services = [service[0] for service in system_state[2]]
        self.topics = [t for t, _ in rospy.get_published_topics()]

        if args.command == 'arm':
            self.call_service('mavros/cmd/arming', service_class=CommandBool,
                              request=CommandBoolRequest(True),
                              agent=args.agent,
                              function=self._are_agents_on_ground)
        elif args.command == 'disarm':
            self.call_service('mavros/cmd/arming', service_class=CommandBool,
                              request=CommandBoolRequest(False),
                              agent=args.agent,
                              function=self._are_agents_on_ground)
        elif args.command == 'land':
            self.call_service('mavros/cmd/land', service_class=CommandTOL,
                              request=CommandTOLRequest(),
                              agent=args.agent)
        elif args.command == 'offboard':
            self.call_service('mavros/set_mode', service_class=SetMode,
                              request=SetModeRequest(custom_mode='OFFBOARD'),
                              agent=args.agent)
        elif args.command == 'takeoff':
            # Takeoff is a combination of takeoff and arming!
            n = rospy.get_param('/gcs/n')
            altitude = rospy.get_param('/gcs/origin_altitude')
            agents = [args.agent] if args.agent is not None else list(range(1, n + 1))
            for agent in agents:
                template = '/drone_{}/mavros/home_position/home'
                home_position = rospy.wait_for_message(template.format(agent), HomePosition)
                request = CommandTOLRequest()
                request.latitude = home_position.geo.latitude
                request.longitude = home_position.geo.longitude
                request.altitude = altitude
                request.yaw = 0.5 * np.pi
                self.call_service('mavros/cmd/takeoff', service_class=CommandTOL,
                                  request=request,
                                  agent=agent)
            self.call_service('mavros/cmd/arming', service_class=CommandBool,
                              request=CommandBoolRequest(True),
                              agent=args.agent,
                              function=self._are_agents_on_ground)

    def loginfo(self, *args, **kwargs):
        print('Swarm control: ', *args, **kwargs)

    def logerr(self, *args, **kwargs):
        print('Swarm control:', *args, file=sys.stderr, **kwargs)

    def _are_agents_on_ground(self):
        topics = [t for t in self.topics if 'mavros/extended_state' in t]
        for topic in topics:
            extended = ExtendedState()
            extended = rospy.wait_for_message(topic, ExtendedState)
            drone = get_drone(topic)
            if extended.landed_state != extended.LANDED_STATE_ON_GROUND:
                rospy.logerr('{} disarm failed. Vehicle not on ground.'.format(drone))
                return False
        return True

    def call_service(self, name, service_class, request=None, agent=None,
                     function=None):

        # Call function and return on False
        if function is not None and not function():
            return

        services = [s for s in self.services if name in s]

        # Filter services by agent
        if agent is not None:
            services = [s for s in services if '/drone_{}/'.format(agent) in s]

        # Call each service
        for service in services:
            rospy.wait_for_service(service, timeout=self.timeout)
            service_proxy = rospy.ServiceProxy(service, service_class)
            response = service_proxy.call(request)
            # Either `success` or `mode_sent` field must be set to true!
            if getattr(response, 'success', False) or getattr(response, 'mode_sent', False):
                self.loginfo('{} success.'.format(service))
            else:
                self.logerr('{} failed.'.format(service))


def main():
    SwarmControl().run()


if __name__ == '__main__':
    main()
