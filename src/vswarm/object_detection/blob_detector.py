import cv2 as cv
from geometry_msgs.msg import Pose2D
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

THRESHOLD_MAX = 255
THRESHOLD = 240


class BlobDetector:

    def __init__(self):
        pass

    def detect_multi(self, images):

        detections_list = []
        for image in images:
            detections = self.detect(image)
            detections_list.append(detections)

        return detections_list

    def detect(self, image):

        # Convert to grayscale if needed
        if image.ndim == 3:
            image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        image_height, image_width = image.shape
        image_area = image_height * image_width

        # Apply (inverse) binary threshold to input image
        mask = cv.threshold(image, THRESHOLD, THRESHOLD_MAX, cv.THRESH_BINARY_INV)[1]

        # Dilate mask to find more reliable contours
        # kernel = np.ones((5, 5), np.uint8)
        # mask_dilated = cv.dilate(mask, kernel, iterations=1)

        # Find external approximate contours in dilated mask
        contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL,
                                              cv.CHAIN_APPROX_SIMPLE)

        # Filter out contours that don't qualify as a detection
        detections = []
        for contour in contours:
            # Filer out if the contour touches the image border
            x, y, w, h = cv.boundingRect(contour)
            if x == 0 or y == 0 or x + w == image_width or y + h == image_height:
                continue
            # Filter out if the contour is too small
            if cv.contourArea(contour) < 1e-4 * image_area:
                continue
            detections.append((x, y, w, h))

        # Fill detections msg
        detection_array_msg = Detection2DArray()
        for detection in detections:
            x, y, w, h = detection

            center_x = x + w / 2.
            center_y = y + h / 2.
            bbox = BoundingBox2D()
            bbox.center = Pose2D(x=center_x, y=center_y, theta=0)
            bbox.size_x = w
            bbox.size_y = h

            object_hypothesis = ObjectHypothesisWithPose()
            object_hypothesis.id = 0
            object_hypothesis.score = 1.0

            detection_msg = Detection2D()
            detection_msg.bbox = bbox
            detection_msg.results.append(object_hypothesis)

            detection_array_msg.detections.append(detection_msg)

        return detection_array_msg
