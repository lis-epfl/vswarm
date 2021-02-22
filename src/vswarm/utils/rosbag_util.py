#!/usr/bin/env python

import os
import rospy
import subprocess
import rosnode
import shlex


class RosbagRecorder:

    def __init__(self, output_dir='~/rosbags'):
        self.node_name = 'rosbag_record'
        self.output_dir = os.path.expanduser(output_dir)
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        self.process = None

    def record(self, regex, bag_name='rosbag'):
        if self.process:
            rospy.logerr('Rosbag already recording...')
            return False
        rosbag_record = 'rosbag record'
        regex_arg = '--regex "{}"'.format(regex)
        command = '{} -O {} -q {} __name:={}'.format(rosbag_record,
                                                     bag_name,
                                                     regex_arg,
                                                     self.node_name)
        command = shlex.split(command)
        self.process = subprocess.Popen(command,
                                        cwd=self.output_dir)
        return self.process is not None

    def stop(self):
        if not self.process:
            rospy.logerr('No rosbag running...')
            return
        self.process = None
        nodes = []
        for node_name in rosnode.get_node_names():
            if self.node_name in node_name:
                nodes.append(node_name)
        success_list, _ = rosnode.kill_nodes(nodes)
        return len(success_list) > 0


def main():
    RosbagRecorder()


if __name__ == '__main__':
    main()
