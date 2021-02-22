#!/usr/bin/env python3

import argparse
import glob
import json
import os

import cv2 as cv
import numpy as np

KEY_YES = 121
KEY_NO = 110
KEY_B = 98
KEY_ESCAPE = 27
KEY_Q = 113
WINDOW_NAME = 'background_subtraction'


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

    subtractor = cv.createBackgroundSubtractorKNN(history=10,
                                                  dist2Threshold=500.,
                                                  detectShadows=True)

    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))

    if args.gui:
        cv.namedWindow(WINDOW_NAME, flags=cv.WINDOW_NORMAL)

    for i, image_filename in enumerate(image_filenames):

        image_color = cv.imread(image_filename, cv.IMREAD_COLOR)
        image_gray = cv.cvtColor(image_color, cv.COLOR_BGR2GRAY)

        height, width = image_gray.shape

        # Mask has values 0 (background), 127 (shadow) and 255 (foreground)
        mask = subtractor.apply(image_gray)
        # Shadow is considered background so filter it out
        mask[mask == 127] = 0

        mask = cv.dilate(mask, kernel, iterations=20)
        contours, hierarchy = cv.findContours(mask, mode=cv.RETR_EXTERNAL,
                                              method=cv.CHAIN_APPROX_SIMPLE)

        # Create empty label dict
        label_dict = get_label_dict(image_filename, width, height)

        # Add detections to label dict (see 'shapes' list)
        if len(contours) > 0:

            # Get only biggest contour area and compute its area
            contour = max(contours, key=cv.contourArea)
            contour_area = cv.contourArea(contour)

            # Only count detections within specified area
            contour_area_max = width * height * 0.9
            contour_area_min = width * height * 0.001
            if contour_area > contour_area_min and contour_area < contour_area_max:

                x, y, w, h = cv.boundingRect(contour)
                box_area = w * h
                area = width * height

                if args.verbose:
                    print(f'Frame {i + 1}')
                    print(f'    filename: {image_filename}')
                    print(f'    bbox: (x: {x} y: {y} w: {w} h: {h})')
                    print(f'    area: {round(100 * box_area / area, 2)} %')

                # Fill in the min/max values of the measured width and height [pixels]
                measured_width_min = 79
                measured_width_max = 296
                measured_height_min = 74
                measured_height_max = 219

                # Fill in the min/max values of the actual width and height [pixels]
                actual_width_min = 42
                actual_width_max = 263
                actual_height_min = 38
                actual_height_max = 179

                # Normalize measured value to range 0-1
                width_norm = np.clip((w - measured_width_min) / (measured_width_max - measured_width_min), 0, 1)
                height_norm = np.clip((h - measured_height_min) / (measured_height_max - measured_height_min), 0, 1)

                # Re-normalize to actual range (from actual values)
                wnew = int(round(width_norm * (actual_width_max - actual_width_min)) + actual_width_min)
                hnew = int(round(height_norm * (actual_height_max - actual_height_min)) + actual_height_min)

                cx, cy = round(x + (w / 2.)), round(y + (h / 2.))
                xnew, ynew = int(round(cx - (wnew / 2.))), int(round(cy - (hnew / 2.)))

                # Add bounding box annotation to image
                image_color = cv.rectangle(image_color,
                                           pt1=(x, y),
                                           pt2=(x + w, y + h),
                                           color=(0, 0, 255),
                                           thickness=2)
                image_color = cv.rectangle(image_color,
                                           pt1=(xnew, ynew),
                                           pt2=(xnew + wnew, ynew + hnew),
                                           color=(0, 255, 0),
                                           thickness=2)

                # Add shape according to bounding box
                shape = get_shape_dict(xnew, ynew, wnew, hnew)
                label_dict['shapes'].append(shape)

        key = KEY_YES
        if args.gui:
            mask_color = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
            cv.imshow(WINDOW_NAME, np.concatenate([image_color, mask_color], axis=0))
            key = cv.waitKeyEx(-1)  # Similar to cv.waitKey except return full key code!
            if key == KEY_ESCAPE or key == KEY_Q:
                break
            elif key == KEY_B:
                cv.imwrite('mask.png', mask_color)
                cv.imwrite('image.png', image_color)

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
