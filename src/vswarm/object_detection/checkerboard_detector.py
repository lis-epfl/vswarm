import cv2 as cv
import cv_bridge
import numpy as np
from vision_msgs.msg import Detection2DArray

from . import util

CRITERIA = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
WIN_SIZE = (9, 9)


class CheckerboardDetector:

    def __init__(self, checkerboard_shape,
                 win_size=WIN_SIZE,
                 criteria=CRITERIA):
        self.checkerboard_shape = checkerboard_shape
        self.win_size = win_size
        self.criteria = criteria
        self.bridge = cv_bridge.CvBridge()

    def detect(self, image):

        detection_array_msg = Detection2DArray()

        found, corners = cv.findChessboardCorners(image,
                                                  self.checkerboard_shape)

        if not found:
            return detection_array_msg

        # Refines corners based on criteria
        corners = cv.cornerSubPix(image, corners=corners, winSize=self.win_size,
                                  zeroZone=(-1, -1), criteria=CRITERIA)

        # Contains N x D (N = number of corners, D = dimensions)
        corners = np.array(corners).squeeze()

        detection_msg = util.corners_to_detection_2d(corners)

        detection_array_msg.detections.append(detection_msg)

        return detection_array_msg
