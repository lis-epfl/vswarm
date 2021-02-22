from __future__ import division, print_function

import os
import sys
import time

import cv2 as cv
import numpy as np
import torch
from geometry_msgs.msg import Pose2D
from torchvision import transforms
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

from .util import rescale_boxes


class YoloDetector:

    def __init__(self, checkpoint_path, config_path, use_gpu=True,
                 confidence_threshold=0.5, iou_threshold=0.5, grayscale=False,
                 verbose=False, half=False):

        self.use_gpu = use_gpu
        # Image size: height x width
        self.image_size = (384, 512)
        self.grayscale = grayscale
        self.verbose = verbose
        self.half = half
        self.checkpoint_path = checkpoint_path
        self.config_path = config_path

        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold

        dirname = os.path.realpath(os.path.dirname(__file__))

        if 'yolov3' in self.config_path:
            # Add toplevel repo to path such that subsequent imports work
            sys.path.append(os.path.join(dirname, 'models', 'yolov3'))
            from .models.yolov3.models import Darknet
            from .models.yolov3.utils.utils import non_max_suppression
            self.model = Darknet(self.config_path, img_size=self.image_size)
        else:
            raise ValueError('Unrecognized detector type for config: {}'.format(self.config_path))

        self.non_max_suppression = non_max_suppression

        if self.half:
            self.model = self.model.half()

        self.has_gpu = torch.cuda.is_available()
        self.device = torch.device('cpu')
        if self.has_gpu and self.use_gpu:
            self.device = torch.device('cuda:0')

        if checkpoint_path.endswith('.pt'):
            checkpoint = torch.load(checkpoint_path, map_location=self.device, encoding='latin1')['model']
            self.model.load_state_dict(checkpoint)
        else:
            extension = checkpoint_path.split('.')[-1]
            print('Unknown checkpoint extension: {}. Exiting.'.format(extension),
                  file=sys.stderr)
            exit(1)

        self.model.fuse()

        if self.has_gpu and self.use_gpu:
            self.model = self.model.cuda()

        self.model.eval()

        self.transform = transforms.Compose([
            transforms.ToTensor()
        ])

    def time_synchronized(self):
        torch.cuda.synchronize() if torch.cuda.is_available() else None
        return time.time()

    def detect_multi(self, images):

        image_shape = images[0].shape

        images_list = []
        for image in images:
            image = cv.resize(image, self.image_size[::-1])  # cv uses width x height!
            image = image[..., np.newaxis]
            if not self.grayscale and image.shape[-1] == 1:
                image = cv.cvtColor(image, cv.COLOR_GRAY2RGB)
            image_torch = self.transform(image)[np.newaxis, ...]
            images_list.append(image_torch)

        images_torch = torch.cat(images_list, 0)

        if self.half:
            images_torch = images_torch.half()

        if self.has_gpu and self.use_gpu:
            images_torch = images_torch.cuda()

        with torch.no_grad():
            # Raw detection is a (N x 8190 x num_classes) tensor
            start_time_inf = self.time_synchronized()
            raw_detections, _ = self.model(images_torch)
            elapsed_time_inf = self.time_synchronized() - start_time_inf
            if self.verbose:
                print('time (inf): {:.4f}s'.format(elapsed_time_inf))

        if self.half:
            raw_detections = raw_detections.float()

        start_time_nms = self.time_synchronized()
        detections_torch = self.non_max_suppression(raw_detections.float(),
                                                    conf_thres=self.confidence_threshold,
                                                    iou_thres=self.iou_threshold)
        elapsed_time_nms = self.time_synchronized() - start_time_nms
        if self.verbose:
            print('time (nms): {:.4f}s'.format(elapsed_time_nms))

        if self.verbose:
            print('time (tot): {:.4f}s\n'.format(elapsed_time_inf + elapsed_time_nms))

        detection_array_list = []
        for detection_torch in detections_torch:

            detection_array_msg = Detection2DArray()

            if detection_torch is not None:

                # Rescale bounding boxes back to original (old) image size
                # shape_old: Shape of images as they come in
                # shape_new: Shape of images used for inference
                detection_torch = rescale_boxes(detection_torch,
                                                shape_old=image_shape[:2],
                                                shape_new=self.image_size)
                detections = detection_torch.cpu().numpy()

                for i, detection in enumerate(detections):
                    x1, y1, x2, y2, object_conf, class_pred = detection

                    class_id = int(class_pred)
                    # class_name = self.classes[class_id]

                    size_x = (x2 - x1)
                    size_y = (y2 - y1)
                    x = x1 + (size_x / 2.0)
                    y = y1 + (size_y / 2.0)
                    bbox = BoundingBox2D()
                    bbox.center = Pose2D(x=x, y=y, theta=0)
                    bbox.size_x = size_x
                    bbox.size_y = size_y

                    object_hypothesis = ObjectHypothesisWithPose()
                    object_hypothesis.id = class_id
                    object_hypothesis.score = object_conf

                    detection_msg = Detection2D()
                    detection_msg.bbox = bbox
                    detection_msg.results.append(object_hypothesis)

                    detection_array_msg.detections.append(detection_msg)

            detection_array_list.append(detection_array_msg)

        return detection_array_list
