#!/usr/bin/env python
from __future__ import division, print_function

import argparse
import os

from tqdm import tqdm

import cv2
import rosbag
from cv_bridge import CvBridge

parser = argparse.ArgumentParser(description='Extract images from rosbag.',
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('bag', help='Input rosbag')
parser.add_argument('topic', help='Image topic')
format_choices = ['png', 'jpg']
parser.add_argument('-f', '--format', type=str.lower, choices=format_choices,
                    default='png', help='Image format')
parser.add_argument('-o', '--output', type=str, default=os.getcwd(),
                    help='Output directory.')

args = parser.parse_args()

args.bag = os.path.expanduser(args.bag)
args.output = os.path.expanduser(args.output)

print('Extracting images from {} to {}'.format(args.bag, args.output))

bag = rosbag.Bag(args.bag, 'r')
bridge = CvBridge()

num_images = 0

for topic, msg, t in tqdm(bag.read_messages(topics=[args.topic]),
                          total=bag.get_message_count(topic_filters=args.topic),
                          unit='msgs'):

    if topic != args.topic:
        continue

    # File name from message number
    filename = 'frame_{:04d}.{}'.format(num_images + 1, args.format)

    # Create subfolders for topic
    path = os.path.join(args.output, topic[1:])
    if not os.path.exists(path):
        os.makedirs(path)

    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    # image = cv2.resize(image, (64, 64))
    cv2.imwrite(os.path.join(path, filename), image)

    num_images += 1
    # print('Saving to {}'.format(os.path.join(path, filename)))

print('{} images saved.'.format(num_images))
