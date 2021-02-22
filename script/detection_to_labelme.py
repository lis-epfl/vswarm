#!/usr/bin/env python3

import argparse
import glob
import json
import os
import sys

import cv2 as cv

KEY_YES = 121
KEY_NO = 110
KEY_ESCAPE = 27
KEY_Q = 113
WINDOW_NAME = 'detection_to_labelme'

dirname = os.path.realpath(os.path.dirname(__file__))
root = os.path.dirname(dirname)
src = os.path.join(root, 'src')

sys.path.append(src)

from object_detection.ultralytics_yolo_detector import UltralyticsYoloDetector


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
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument('data_dir', help='Directory with image data.')
    parser.add_argument('checkpoint_path', type=str, help='Checkpoint')
    parser.add_argument('config_path', type=str, help='Config')
    parser.add_argument('--conf-thresh', type=float, default=0.5,
                        help='Confidence threshold')
    parser.add_argument('--iou-thresh', type=float, default=0.5,
                        help='IOU threshold')
    parser.add_argument('-d', '--dry-run', default=False, action='store_true',
                        help='Do not write json label files')
    parser.add_argument('-o', '--overwrite', default=False, action='store_true',
                        help='Overwrite existing labels')
    parser.add_argument('-v', '--verbose', default=False, action='store_true',
                        help='Verbose output')
    parser.add_argument('-g', '--gui', default=False, action='store_true',
                        help='Show current frame in GUI')
    parser.add_argument('-p', '--no-progress', default=True, action='store_false',
                        help='Disable progress bar')

    args = parser.parse_args()

    if not os.path.isabs(args.data_dir):
        args.data_dir = os.path.abspath(os.path.join(os.getcwd(), args.data_dir))

    if not os.path.isdir(args.data_dir):
        print('Directory does not exist: {}'.format(args.data_dir))
        exit()

    image_extension = 'png'
    label_extension = 'json'
    image_filenames = sorted(glob.glob(args.data_dir + '/*.' + image_extension))

    if not args.no_progress:
        from tqdm import tqdm
        progress_bar = tqdm(total=len(image_filenames), unit='images', dynamic_ncols=True,
                            desc='Progress')

    detector = UltralyticsYoloDetector(checkpoint_path=args.checkpoint_path,
                                       config_path=args.config_path,
                                       confidence_threshold=args.conf_thresh,
                                       iou_threshold=args.iou_thresh)

    if args.gui:
        cv.namedWindow(WINDOW_NAME, flags=cv.WINDOW_NORMAL)

    for i, image_filename in enumerate(image_filenames):

        image_color = cv.imread(image_filename, cv.IMREAD_COLOR)
        image_gray = cv.cvtColor(image_color, cv.COLOR_BGR2GRAY)

        height, width = image_gray.shape

        text = os.path.basename(image_filename)
        org = (int(0.03 * width), height - int(0.03 * height))
        image_color = cv.putText(image_color, text=text, org=org,
                                 fontFace=cv.FONT_HERSHEY_COMPLEX,
                                 fontScale=0.6, color=(255, 255, 255),
                                 thickness=1, lineType=cv.LINE_AA)

        detection_array_msg = detector.detect_multi([image_gray])[0]

        # Create empty label dict
        label_dict = get_label_dict(image_filename, width, height)

        for detection in detection_array_msg.detections:

            cx = int(round(detection.bbox.center.x))
            cy = int(round(detection.bbox.center.y))
            w = int(round(detection.bbox.size_x))
            h = int(round(detection.bbox.size_y))

            x = int(round(cx - (w / 2.0)))
            y = int(round(cy - (h / 2.0)))

            if args.verbose:
                print(f'Frame {i + 1}')
                print(f'    filename: {image_filename}')
                print(f'    bbox: (x: {x} y: {y} w: {w} h: {h})')

            # Add bounding box annotation to image
            image_color = cv.rectangle(image_color,
                                       pt1=(x, y),
                                       pt2=(x + w, y + h),
                                       color=(0, 0, 255),
                                       thickness=2)

            # Add shape according to bounding box
            shape = get_shape_dict(x, y, w, h)
            label_dict['shapes'].append(shape)

        key = KEY_YES
        if args.gui:
            cv.imshow(WINDOW_NAME, image_color)
            key = cv.waitKeyEx(-1)  # Similar to cv.waitKey except return full key code!
            if key == KEY_ESCAPE or key == KEY_Q:
                break

        label_filename = os.path.splitext(image_filename)[0] + '.' + label_extension

        # Only overwrite when flag is set
        if os.path.exists(label_filename) and not args.overwrite:
            print('Label `{}` already exists. Skipping.'.format(label_filename))
            continue

        # Do nothing if dry run flag is set
        if not args.dry_run and key == KEY_YES:
            write_label_json(label_filename, label_dict)

        if not args.no_progress:
            progress_bar.update()


if __name__ == '__main__':
    main()
