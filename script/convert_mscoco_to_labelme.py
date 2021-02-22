#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import argparse
import json
import os

LABEL_DICT = {
    0: 'jetson_lequad'
}


def get_label_dict(image_filename, width, height):
    label_dict = {
        'fillColor': [255, 0, 0],
        'flags': {},
        'imageData': None,
        'imagePath': os.path.basename(image_filename),
        'imageWidth': width,
        'imageHeight': height,
        'lineColor': [0, 255, 0, 128],
        'shapes': [],
        'version': '3.16.2',
    }
    return label_dict


def get_shape_dict(x, y, w, h, label='jetson_lequad'):
    shape_dict = {
        'flags': {},
        'fill_color': None,
        'line_color': None,
        'shape_type': 'rectangle',
        'points': [
            [x, y],
            [x + w, y + h]
        ],
        'label': label
    }
    return shape_dict


def write_label_json(filename, label_dict):
    with open(filename, 'w') as f:
        json.dump(label_dict, f, indent=4, sort_keys=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('label', type=str, help='Input label')
    parser.add_argument('--width', type=int, default=720, help='Image width')
    parser.add_argument('--height', type=int, default=540, help='Image height')
    parser.add_argument('-o', '--output', type=str, default=None, help='Output file name')

    args = parser.parse_args()

    detections = [l.strip() for l in open(args.label, 'r').readlines()]

    image_filename = args.label.replace('.txt', '.png')
    label_dict = get_label_dict(image_filename, args.width, args.height)

    for detection in detections:

        s_class_id, s_x_center, s_y_center, s_width, s_height = detection.split(' ')
        class_id = int(s_class_id)
        x_center_rel = float(s_x_center)
        y_center_rel = float(s_y_center)
        width_rel = float(s_width)
        height_rel = float(s_height)

        x_center_abs = x_center_rel * args.width
        y_center_abs = y_center_rel * args.height

        width = int(width_rel * args.width)
        height = int(height_rel * args.height)

        x = int(x_center_abs - 0.5 * width)
        y = int(y_center_abs - 0.5 * height)

        shape_dict = get_shape_dict(x, y, width, height, label=LABEL_DICT[class_id])

        label_dict['shapes'].append(shape_dict)

    label_filename = image_filename.replace('.png', '.json')
    write_label_json(label_filename, label_dict)

    # s = json.dumps(label_dict, indent=4, sort_keys=True)
    # print(s)


if __name__ == '__main__':
    main()
