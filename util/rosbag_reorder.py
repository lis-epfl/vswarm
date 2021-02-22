#!/usr/bin/env python
import argparse
import os

import rosbag
import rospy
from tqdm import tqdm


def main():

    # Parse and clean command line arguments
    parser = argparse.ArgumentParser(
        description='Reorder rosbag based on header timestamps',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('bag', type=str, help='Input bag file')
    parser.add_argument('--ensure-tf-available', action='store_true', default=False,
                        help='Ensure TF availability with time offset')
    args = parser.parse_args()
    args.bag = os.path.expanduser(args.bag)

    # Outbag filename
    path, basename = os.path.split(args.bag)
    filename, extension = basename.rsplit('.', 1)  # Split filename, ext
    outbag_filename = '{}_ordered.{}'.format(filename, extension)
    outbag_path = os.path.join(path, outbag_filename)

    bag = rosbag.Bag(args.bag)

    # Write ordered rosbag
    with rosbag.Bag(outbag_path, 'w') as outbag:
        for topic, msg, t in tqdm(bag.read_messages(),
                                  total=bag.get_message_count(),
                                  dynamic_ncols=True,
                                  unit='msgs'):
            if topic == '/tf' and msg.transforms:
                timestamp = msg.transforms[0].header.stamp
                # Ensure tf availability: write tfs 1 second ahead of time
                if args.ensure_tf_available:
                    timestamp - rospy.Duration(1)
                outbag.write(topic, msg, timestamp)
            else:
                timestamp = msg.header.stamp if msg._has_header else t
                outbag.write(topic, msg, timestamp)


if __name__ == '__main__':
    main()
