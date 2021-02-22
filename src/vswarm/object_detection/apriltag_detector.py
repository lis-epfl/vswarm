import apriltag
from vision_msgs.msg import Detection2DArray

from . import util


class ApriltagDetector:

    def __init__(self):
        options = apriltag.DetectorOptions(refine_pose=True)
        self.detector = apriltag.Detector(options=options)

    def detect(self, image):

        detection_array_msg = Detection2DArray()

        detection_results = self.detector.detect(image)

        if len(detection_results) == 0:
            return detection_array_msg

        for detection_result in detection_results:

            # Contains N x D (N = number of corners, D = dimensions)
            corners = detection_result.corners.squeeze()

            detection_msg = util.corners_to_detection_2d(corners)

            detection_array_msg.detections.append(detection_msg)

        return detection_array_msg

    def detect_pose(self, image, camera_info, tag_size=1):

        fx = camera_info.K[0]
        cx = camera_info.K[2]
        fy = camera_info.K[4]
        cy = camera_info.K[5]

        camera_params = [fx, fy, cx, cy]

        # Assume there is only one detection
        detection = self.detector.detect(image)[0]

        # Return transform (translation + rotation) and errors (disregard!)
        transform, _, _ = self.detector.detection_pose(detection, camera_params,
                                                       tag_size=tag_size)

        return transform
