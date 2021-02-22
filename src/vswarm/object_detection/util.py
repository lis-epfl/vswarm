from geometry_msgs.msg import Pose2D
from vision_msgs.msg import BoundingBox2D, Detection2D


def corners_to_detection_2d(corners):

    # Find min/max bounding box points
    min_x = corners[:, 0].min()
    max_x = corners[:, 0].max()
    min_y = corners[:, 1].min()
    max_y = corners[:, 1].max()

    # Find size of bounding box in x and y direction
    size_x = max_x - min_x
    size_y = max_y - min_y

    # Find center coordinates
    center_x = min_x + (size_x / 2.0)
    center_y = min_y + (size_y / 2.0)

    # Construct detection message
    center = Pose2D(x=center_x, y=center_y)
    bounding_box = BoundingBox2D(center=center,
                                 size_x=size_x,
                                 size_y=size_y)

    return Detection2D(bbox=bounding_box)


def detection_2d_to_points(detection_msg):

    center_x = detection_msg.bbox.center.x
    center_y = detection_msg.bbox.center.y
    size_x = detection_msg.bbox.size_x
    size_y = detection_msg.bbox.size_y

    # Rectangle drawing function expects integers
    min_x = int(center_x - (size_x / 2.))
    max_x = int(center_x + (size_x / 2.))
    min_y = int(center_y - (size_y / 2.))
    max_y = int(center_y + (size_y / 2.))

    point_min = (min_x, min_y)
    point_max = (max_x, max_y)

    return point_min, point_max


def is_edge_detection(detection_msg, width, height,
                      width_thresh=0.0,
                      height_thresh=0.0):
    """Return True if a detection is on the edge of an image with a threshold.

    Args:
        detection_msg (vision_msgs/Detection2D): Detection message
        width (int): Image width
        height (int): Image height
        width_thresh (float): Relative width threshold [0.0 - 0.5]
        height_thresh (float): Relative height threshold [0.0 - 0.5]

    Returns:
        bool: True, if detection too close to the edge of the image

    """
    if width_thresh < 0.0 or width_thresh > 0.5:
        raise ValueError('Invalid width_thresh. Got {}.'.format(width_thresh))

    if height_thresh < 0.0 or height_thresh > 0.5:
        raise ValueError('Invalid height_thresh. Got {}.'.format(height_thresh))

    point_min, point_max = detection_2d_to_points(detection_msg)
    min_x, min_y = point_min
    max_x, max_y = point_max

    min_width = width_thresh * width
    max_width = (width - min_width)
    min_height = height_thresh * height
    max_height = (height - min_height)

    return min_x <= min_width or min_y <= min_height or \
        max_x >= max_width or max_y >= max_height


def rescale_boxes(boxes, shape_old, shape_new):
    # Rescale coords (xyxy) from shape_new (hw) to shape_old (hw)

    gain_height = shape_new[0] / shape_old[0]
    gain_width = shape_new[1] / shape_old[1]

    pad_y = (shape_new[0] - shape_old[0] * gain_height) / 2
    pad_x = (shape_new[1] - shape_old[1] * gain_width) / 2

    boxes[:, [0, 2]] -= pad_x
    boxes[:, [1, 3]] -= pad_y

    boxes[:, [0, 2]] /= gain_width
    boxes[:, [1, 3]] /= gain_height

    clip_boxes(boxes, shape_old)

    return boxes


def clip_boxes(boxes, shape):
    # Clip bounding xyxy bounding boxes to image shape (height, width)
    boxes[:, 0].clamp_(0, shape[1])  # x1
    boxes[:, 1].clamp_(0, shape[0])  # y1
    boxes[:, 2].clamp_(0, shape[1])  # x2
    boxes[:, 3].clamp_(0, shape[0])  # y2
