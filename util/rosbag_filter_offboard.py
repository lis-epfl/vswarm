#!/usr/bin/env python
import argparse
import os

import rosbag
from tqdm import tqdm


def main():

    # Parse and clean command line arguments
    parser = argparse.ArgumentParser(
        description='Reorder rosbag based on header timestamps',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('bag', type=str, help='Input bag file')
    parser.add_argument('-t', '--topic', type=str,
                        help='State topic to filter on',
                        default='/drone_1/mavros/state')
    args = parser.parse_args()
    args.bag = os.path.expanduser(args.bag)

    # Outbag filename
    path, basename = os.path.split(args.bag)
    filename, extension = basename.rsplit('.', 1)  # Split filename, ext
    outbag_filename = '{}_offboard.{}'.format(filename, extension)
    outbag_path = os.path.join(path, outbag_filename)

    is_offboard = False

    bag = rosbag.Bag(args.bag)

    # Write ordered rosbag
    with rosbag.Bag(outbag_path, 'w') as outbag:
        for topic, msg, t in tqdm(bag.read_messages(),
                                  total=bag.get_message_count(),
                                  dynamic_ncols=True,
                                  unit='msgs'):
            if topic == args.topic:
                is_offboard = (msg.mode == 'OFFBOARD')
            if is_offboard:
                outbag.write(topic, msg, t)


if __name__ == '__main__':
    main()
