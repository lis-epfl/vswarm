#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

"""
Converts labelme labels into MSCOCO labels.

## Directory structure

- dataset:
    - images:
        - subdir:
            - *.png, *.json  # Images and labels (labelme)
    - config:
        - train.txt  # Contains filenames of training images
        - valid.txt  # Contains filenames of validation images
    - labels:  # Needs to exist and will be filled with MSCOCO labels!

## Usage

```bash
cd path/to/dataset
export DIR=images/subdir/
convert_labelme_to_mscoco.py "$DIR" --output "${DIR/images/labels}"
```
"""

import argparse
import glob
import json
import os
import sys

import cv2 as cv
import numpy as np
import tqdm

LABEL_DICT = {
    'jetson_lequad': 0
}

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument('data_dir', help='Directory with json files')
parser.add_argument('-o', '--output', type=str, default=None, help='Label output dir')
parser.add_argument('--show-image', action='store_true', default=False, help='Show images')
parser.add_argument('--dry-run', action='store_true', default=False, help='Dry run')

args = parser.parse_args()

# Convert data_dir to absolute path
if not os.path.isabs(args.data_dir):
    args.data_dir = os.path.join(os.getcwd(), args.data_dir)
args.data_dir = os.path.abspath(args.data_dir)

if args.output is None:
    args.output = args.data_dir
else:
    # Convert output dir to absolute path
    if not os.path.isabs(args.output):
        args.output = os.path.join(os.getcwd(), args.output)
    args.output = os.path.abspath(args.output)

    # Create output directory
    if os.path.exists(args.output):
        print('Output directory exists: {}. Exiting'.format(args.output), file=sys.stderr)
        exit(-1)
    os.makedirs(args.output)

filepaths = sorted(glob.glob(args.data_dir + '/*.json'))

for filepath in tqdm.tqdm(filepaths, unit='labels'):

    filename = os.path.basename(filepath)  # frame_XXXX.json

    if args.show_image:
        imagepath = filepath.replace('.json', '.png')  # frame_XXXX.png
        image = cv.imread(imagepath)

    with open(filepath, 'r') as f:
        json_dict = json.load(f)

    image_height = json_dict['imageHeight']
    image_width = json_dict['imageWidth']
    image_size = np.array([image_width, image_height], dtype=np.float64)
    shapes = json_dict['shapes']

    filecontent = ''

    for shape in shapes:

        assert shape['shape_type'] == 'rectangle'

        class_label = shape['label']
        points = np.array(shape['points'], dtype=np.float64)  # [[x1, y1], [x2, y2]]
        pt1, pt2 = points

        # All pt2 points need to be larger than or equal to pt1 points
        if np.any(pt1 > pt2):
            x1, y1 = pt1
            x2, y2 = pt2
            if x1 > x2:
                x1, x2 = x2, x1
            if y1 > y2:
                y1, y2 = y2, y1
            pt1 = np.array([x1, y1])
            pt2 = np.array([x2, y2])

        # pt1 (x1, y1), pt2 (x2, y2) -> center (x, y), size (w, h)
        bbox_size = (pt2 - pt1)  # [size_x, size_y]
        bbox_center = pt1 + (bbox_size / 2.)  # [center_x, center_y]

        # Absolute pixel size -> normalized [0 - 1]
        bbox_size_norm = bbox_size / image_size
        bbox_center_norm = bbox_center / image_size

        # Do calculations in reverse to test

        # Normalized -> absolute
        bbox_size_abs = bbox_size_norm * image_size
        bbox_center_abs = bbox_center_norm * image_size

        # center (x, y), size(w, h) -> pt1 (x1, y1), pt2 (x2, y2)
        pt1_abs = bbox_center_abs - (bbox_size_abs / 2.)
        pt2_abs = bbox_center_abs + (bbox_size_abs / 2.)

        assert np.allclose(pt1, pt1_abs)
        assert np.allclose(pt2, pt2_abs)

        if args.show_image:
            point1 = tuple(np.round(pt1_abs).astype(np.int32))
            point2 = tuple(np.round(pt2_abs).astype(np.int32))
            image = cv.rectangle(image, tuple(point1), tuple(point2),
                                 color=(0, 255, 0), thickness=2)
            # labelimagepath = imagepath.replace('.png', '_labelimage.png')
            # cv.imwrite(labelimagepath, image)

        try:
            class_id = LABEL_DICT[class_label]
        except KeyError:
            print('Key error in ', filepath)
        x_center, y_center = bbox_center_norm
        width, height = bbox_size_norm

        template = '{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}'
        template = template.format(class_id=class_id,
                                   x_center=x_center,
                                   y_center=y_center,
                                   width=width,
                                   height=height)

        filecontent += template + '\n'

    if args.show_image:
        cv.imshow('Image', image)
        cv.waitKey(-1)

    outfilename = filename.replace('.json', '.txt')  # frame_XXXX.txt
    outfilepath = os.path.join(args.output, outfilename)

    if not args.dry_run:
        with open(outfilepath, 'w') as f:
            f.write(filecontent)
    else:
        print(filecontent, '>>', outfilepath)
