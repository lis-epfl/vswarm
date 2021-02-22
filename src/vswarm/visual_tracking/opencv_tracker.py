from __future__ import division, print_function

from vision_msgs.msg import Detection2DArray, Detection2D

import cv2 as cv


class OpenCVTracker:

    def __init__(self, image, detections, name='CSRT'):
        self.tracker = cv.MultiTracker_create()
        for detection in detections:
            x = detection.bbox.center.x
            y = detection.bbox.center.y
            w = detection.bbox.size_x
            h = detection.bbox.size_y
            bbox = (x, y, w, h)
            self.tracker.add(self._get_tracker(name), image, bbox)

    def track(self, image):
        detections = Detection2DArray()
        success, boxes = self.tracker.update(image)
        if not success:
            return detections
        for box in boxes:
            x, y, w, h = box
            detection = Detection2D()
            detection.bbox.center.x = x
            detection.bbox.center.y = y
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            detections.detections.append(detection)
        return detections

    def _get_tracker(self, name):
        tracker = None
        if name == 'Boosting':
            tracker = cv.TrackerBoosting_create()
        elif name == 'MIL':
            tracker = cv.TrackerMIL_create()
        elif name == 'KCF':
            tracker = cv.TrackerKCF_create()
        elif name == 'TLD':
            tracker = cv.TrackerTLD_create()
        elif name == 'MedianFlow':
            tracker = cv.TrackerMedianFlow_create()
        elif name == 'GOTURN':
            tracker = cv.TrackerGOTURN_create()
        elif name == 'MOSSE':
            tracker = cv.TrackerMOSSE_create()
        elif name == 'CSRT':
            tracker = cv.TrackerCSRT_create()
        else:
            raise RuntimeError('Unrecognized tracker type: {}'.format(name))
        return tracker
