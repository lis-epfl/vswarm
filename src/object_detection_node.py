#!/usr/bin/env python3

"""
This node detects objects in images from multiple cameras.

Inputs: images [sensor_msgs/Image]
Output: detections [vision_msgs/Detection2DArray]
"""

from __future__ import absolute_import, division, print_function

import os
import sys

import cv2 as cv
import numpy as np
import rospy
from cv_bridge import CvBridge
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from vswarm.cfg import ObjectDetectionNodeConfig
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from vision_msgs.msg import Detection2DArray

from vswarm.visual_tracking.opencv_tracker import OpenCVTracker
from vswarm.object_detection import util


class ObjectDetectionNode:

    def __init__(self):

        self.node_name = 'object_detection_node'
        rospy.init_node(self.node_name, argv=sys.argv)

        # Parameters
        self.type = rospy.get_param('~type', default='yolo')
        self.checkpoint = rospy.get_param('~checkpoint', default=None)
        self.config = rospy.get_param('~config', default=None)
        self.grayscale = rospy.get_param('~grayscale', default=False)
        self.verbose = rospy.get_param('~verbose', default=False)
        self.namespace = rospy.get_namespace().split('/')[1]

        # Choose detector type
        rospy.loginfo('Loading detector: {}'.format(self.type))
        if self.type == 'apriltag':
            from vswarm.object_detection.apriltag_detector import ApriltagDetector
            self.detector = ApriltagDetector()
        elif self.type == 'blob':
            from vswarm.object_detection.blob_detector import BlobDetector
            self.detector = BlobDetector()
        elif self.type == 'checkerboard':
            from vswarm.object_detection.checkerboard_detector import CheckerboardDetector
            self.detector = CheckerboardDetector(checkerboard_shape=(8, 6))
        elif self.type == 'yolo':
            from vswarm.object_detection.yolo_detector import YoloDetector
            self.detector = YoloDetector(checkpoint_path=self.checkpoint,
                                         config_path=self.config,
                                         grayscale=self.grayscale,
                                         verbose=self.verbose)
            rospy.loginfo('Loaded config: {}'.format(os.path.basename(self.config)))
        else:
            rospy.logerr('Unrecognized detector type: {}'
                         .format(self.type))
            exit(1)
        rospy.loginfo('Detector loaded: {}'.format(self.type))

        # Try connecting to global dynamic reconfigure server, otherwise create local one
        self.config = {}
        try:
            self.client = Client('/gcs/object_detection_config_server',
                                 config_callback=self.config_callback,
                                 timeout=1)
            rospy.loginfo('Connected to remote dynamic reconfigure server.')
        except rospy.ROSException:
            rospy.logwarn('Failed to connect to dynamic reconfigure server.')
            rospy.loginfo('Connected to local dynamic reconfigure server.')
            self.server = Server(ObjectDetectionNodeConfig, callback=self.config_callback)
            self.client = Client(self.node_name)
        self.config = self.client.get_configuration()

        self.bridge = CvBridge()

        # Visual tracking
        self.track_length = 0
        self.trackers = []
        self.initial_detections_list = []

        # Publishers
        detections_pub_topic = self.node_name + '/detections'
        self.detections_pub = rospy.Publisher(detections_pub_topic, Detection2DArray,
                                              queue_size=1)

        num_detections_pub_topic = self.node_name + '/num_detections'
        self.num_detections_pub = rospy.Publisher(num_detections_pub_topic, Int32,
                                                  queue_size=1)

        self.input_images = rospy.get_param('~input_images')

        image_topic = self.node_name + '/detections/image_raw'
        self.image_publisher = rospy.Publisher(image_topic, Image, queue_size=1)

        self.image_messages = [None for _ in self.input_images]
        self.image_updated = [False for _ in self.input_images]
        self.image_subscribers = []
        for i, topic in enumerate(self.input_images):

            # Subscriber
            subscriber = rospy.Subscriber(topic, Image,
                                          callback=self.image_callback,
                                          callback_args=i,
                                          queue_size=1,
                                          buff_size=2 ** 24)
            self.image_subscribers.append(subscriber)

    def config_callback(self, config, level=None):
        for name, new_value in config.items():
            if name == 'groups' or name not in self.config:
                continue
            old_value = self.config[name]
            if old_value != new_value:
                rospy.loginfo('Update `{}`: {} -> {}'.format(name, old_value, new_value))
        return self.update_config(config)

    def update_config(self, config):
        self.config = config

        # Update underlying detector
        if hasattr(self.detector, 'confidence_threshold'):
            self.detector.confidence_threshold = self.config['confidence_threshold']
        if hasattr(self.detector, 'iou_threshold'):
            self.detector.iou_threshold = self.config['iou_threshold']

        return config

    def image_callback(self, image_msg, i):
        rospy.loginfo_once('Image callback')
        self.image_messages[i] = image_msg
        self.image_updated[i] = True
        if all(self.image_updated):
            self.image_updated = [False for _ in self.input_images]
            self.callback(*self.image_messages[:])

    def callback(self, *image_msgs):

        # Extract width and height from image message
        width, height = image_msgs[0].width, image_msgs[0].height

        # IMPORTANT: if frame_id of image_msg is not fully qualified, we'll do it here
        # This is needed to associate the image with the proper TF and calibration
        # The fully qualified frame_id is: drone_<N>/camera_<direction>_optical
        for image_msg, topic in zip(image_msgs, self.input_images):
            # In reality, the frame_id does not start with 'drone_'
            if not image_msg.header.frame_id.startswith('drone_'):
                camera_name = topic.split('/')[-2]
                image_msg.header.frame_id = '{}/{}_optical'.format(self.namespace, camera_name)
            # In simulation, the frame_id ends with '_link' which we don't want
            if image_msg.header.frame_id.endswith('_link'):
                frame_id = image_msg.header.frame_id.replace('_link', '_optical')
                image_msg.header.frame_id = frame_id

        # Convert ROS image messages to grayscale numpy images
        images = []
        for image_msg in image_msgs:
            image_raw = self.bridge.imgmsg_to_cv2(image_msg)
            if image_raw.ndim == 3:
                image_raw = cv.cvtColor(image_raw, cv.COLOR_BGR2GRAY)
            images.append(image_raw)

        # Optional: introduce processing delay
        if self.config['delay'] > 0.0:
            rospy.sleep(self.config['delay'])

        if self.track_length > self.config['max_track_length']:
            self.track_length = 0

        # Detect or track
        detection_array_msg_list = []
        if self.config['use_visual_tracking'] and self.track_length > 0:
            rospy.logdebug('Tracking: {}/{}'.format(self.track_length, self.config['max_track_length']))
            for i, (tracker, image, dets) in enumerate(zip(self.trackers, images, self.initial_detections_list)):
                if tracker is not None:
                    tracked = tracker.track(image)
                    if len(tracked.detections) < len(dets.detections):
                        rospy.logdebug('Lost track!')
                        self.trackers[i] = None
                else:
                    tracked = Detection2DArray()
                detection_array_msg_list.append(tracked)
            self.track_length += 1
        else:
            # Input: list of images, output: list of detection arrays
            rospy.logdebug('Detection')
            detection_array_msg_list = self.detector.detect_multi(images)

        # Create trackers only for images for which we have detections
        if self.config['use_visual_tracking'] and self.track_length == 0:
            self.trackers = []
            self.initial_detections_list = detection_array_msg_list
            rospy.logdebug('Tracking: feeding detection to tracker')
            for image, detection_array_msg in zip(images, self.initial_detections_list):
                if len(detection_array_msg.detections) > 0:
                    tracker = OpenCVTracker(image, detection_array_msg.detections)
                else:
                    tracker = None
                self.trackers.append(tracker)
            self.track_length += 1

        # Filter out edge detections and (optionally) add false negatives
        out_detection_array_msg_list = []
        for detection_array_msg, image_msg in zip(detection_array_msg_list, image_msgs):
            out_detection_array = Detection2DArray()
            for detection_msg in detection_array_msg.detections:

                # Reject detections based distance to the image edges
                if util.is_edge_detection(detection_msg, width, height,
                                          width_thresh=0.01, height_thresh=0.01):
                    continue

                # Drop detections with false negative probability
                if bool(np.random.binomial(1, p=self.config['false_negative_prob'])):
                    continue

                # Add out detection with image header to detections
                # We add the image header to match image to detection later on
                out_detection = detection_msg
                out_detection.header = image_msg.header
                out_detection_array.detections.append(detection_msg)

            out_detection_array_msg_list.append(out_detection_array)

        # Aggregate results from multiple detection arrays into one (with proper frame_ids)
        out_detection_array_msg = Detection2DArray()
        out_detection_array_msg.header.stamp = rospy.Time.now()
        # The following frame_id does not actually exist (check individual detections!)
        frame_id = image_msgs[0].header.frame_id.split('/')[0] + '/camera_array'
        out_detection_array_msg.header.frame_id = frame_id
        for detection_array_msg in out_detection_array_msg_list:
            for detection_msg in detection_array_msg.detections:
                out_detection_array_msg.detections.append(detection_msg)

        # Publish detections
        self.detections_pub.publish(out_detection_array_msg)
        self.num_detections_pub.publish(Int32(len(out_detection_array_msg.detections)))

        # Publish image annotated with detections (optionally)
        if self.config['publish_image']:
            images_annotated = []
            for image, detection_array_msg in zip(images, out_detection_array_msg_list):
                image_annotated = cv.cvtColor(image, cv.COLOR_GRAY2BGR)
                for detection_msg in detection_array_msg.detections:
                    image_annotated = self._draw_detection(image_annotated, detection_msg)
                images_annotated.append(image_annotated)

            # Concatenate images along width axis
            # image_concat = self._make_collage(images_annotated)
            image_concat = self._make_image_strip(images_annotated)

            # Publish image with detection overlay
            image_detection_msg = self.bridge.cv2_to_imgmsg(image_concat, encoding='bgr8')
            image_detection_msg.header.stamp = rospy.Time.now()
            self.image_publisher.publish(image_detection_msg)

        # Keep previous detection list for tracking
        self.previous_detection_list = out_detection_array_msg

    def _draw_detection(self, image, detection_msg):
        """Draw detection bounding box and text"""
        pt1, pt2 = util.detection_2d_to_points(detection_msg)
        color = (0, 0, 255) if self.track_length == 0 else (0, 255, 0)
        image = cv.rectangle(image, pt1=pt1, pt2=pt2, color=color, thickness=2)
        if len(detection_msg.results) != 0:
            object_hypothesis = detection_msg.results[0]
            score = object_hypothesis.score * 100
            # name = 'Drone'  # object_hypothesis.id  # TODO: human-readable name
            text = '{score}%'.format(score=int(round(score)))
            x, y = pt1
            org = (x, y - 5)
            image = cv.putText(image, text=text, org=org,
                               fontFace=cv.FONT_HERSHEY_COMPLEX,
                               fontScale=0.4, color=color,
                               thickness=1, lineType=cv.LINE_AA)
        return image

    def _make_image_strip(self, image_list):
        return np.concatenate(image_list, axis=1)

    def _make_collage(self, image_list):
        height, width, channels = image_list[0].shape
        dtype = image_list[0].dtype
        size = int(np.ceil(np.sqrt(len(image_list))))  # Grid: size x size
        collage = np.zeros((size * height, size * width, channels), dtype=dtype)
        index = 0
        for col in range(size):
            for row in range(size):
                if index > len(image_list):
                    break
                h, w = row * height, col * width
                collage[h:h + height, w:w + width, :] = image_list[index]

                # Add text
                text = self.input_images[index]
                org = (w + int(0.03 * width), h + height - int(0.03 * height))
                collage = cv.putText(collage, text=text, org=org,
                                     fontFace=cv.FONT_HERSHEY_COMPLEX,
                                     fontScale=0.6, color=(255, 255, 255),
                                     thickness=1, lineType=cv.LINE_AA)
                index += 1
        return collage


def main():
    ObjectDetectionNode()
    rospy.spin()


if __name__ == '__main__':
    main()
