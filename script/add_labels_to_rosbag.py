#!/usr/bin/env python3
from __future__ import division, print_function

import argparse
import glob
import os
import sys

import rosbag
from geometry_msgs.msg import Pose2D
from tqdm import tqdm
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument('bag', type=str, help='Bag file')
parser.add_argument('topic', type=str, help='Image topic')
parser.add_argument('label_dir', type=str, help='Label directory')
parser.add_argument('-o', '--output-bag', default='output.bag', type=str,
                    help='Output bag')
parser.add_argument('-d', '--detection-topic', default='/detections', type=str,
                    help='Detection topic')
parser.add_argument('-v', '--verbose', action='store_true', default=False,
                    help='Verbose output')

args = parser.parse_args()

args.bag = os.path.abspath(args.bag)
if not os.path.exists(args.bag):
    print('Bag file {} not found. Exiting.'.format(args.bag), file=sys.stderr)
    exit(-1)
args.output_bag = os.path.abspath(args.output_bag)
if os.path.exists(args.output_bag):
    print('Output bag {} already exists. Exiting.'.format(args.output_bag),
          file=sys.stderr)
    exit(-2)
args.label_dir = os.path.abspath(args.label_dir)
if not os.path.exists(args.label_dir) or not os.path.isdir(args.label_dir):
    print('Label directory {} does not exist. Exiting.'.format(args.label_dir),
          file=sys.stderr)
    exit(-3)

# Reading labels
label_filenames = sorted(glob.glob(args.label_dir + os.sep + '*.txt'))
label_strings = {}
labels = {}
for filepaths in label_filenames:
    label_file = open(filepaths, 'r').read()
    if label_file.strip() == '':  # TODO: may be error-prone
        continue
    frame_id = int(os.path.basename(filepaths).split('.')[0].split('_')[-1])
    label_list = []
    for label in label_file.split('\n'):
        if label.strip() == '':  # TODO: may be error-prone
            continue
        class_id, center_x, center_y, width, height = label.strip().split(' ')
        class_id = int(class_id)
        center_x, center_y = float(center_x), float(center_y)
        width, height = float(width), float(height)
        label_list.append((class_id, center_x, center_y, width, height))
    labels[frame_id] = label_list

if args.verbose:
    print('Parsed {} labels.'.format(len(labels)))

# Write new bag
bag = rosbag.Bag(args.bag, mode='r')
outbag = rosbag.Bag(args.output_bag, mode='w')

progress_bar = tqdm(total=bag.get_message_count(topic_filters=args.topic),
                    dynamic_ncols=True, unit='msgs')

index = 1
for topic, msg, t in bag.read_messages():

    # If image topic, add label message at same time step
    if topic == args.topic:

        progress_bar.update()

        image_width, image_height = float(msg.width), float(msg.height)

        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        label_list = []
        try:
            label_list = labels[index]
        except KeyError:
            print('Image {} has no label. Skipping.'.format(index), file=sys.stderr)

        for label in label_list:

            _, center_x, center_y, width, height = label

            # Scale by image height and width
            center_x *= image_width
            center_y *= image_height
            width *= image_width
            height *= image_height

            center = Pose2D(x=center_x, y=center_y)
            bbox = BoundingBox2D(center=center, size_x=width, size_y=height)
            detection = Detection2D(bbox=bbox)

            detection_array_msg.detections.append(detection)

        outbag.write(args.detection_topic, detection_array_msg, t)

        index += 1

    # Write everything to outbag
    outbag.write(topic, msg, t)

bag.close()
outbag.close()
