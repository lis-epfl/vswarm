#!/usr/bin/env python3
"""
This node tests image subscriber lag.
Simulated scenario: the image processing in the image callback takes longer than the publishing rate of incoming images.
If queue sizes are set appropriately in the publisher and subscriber, we would expect messages to be dropped.
To run the tests:

```bash
rosrun image_publisher image_publisher /path/to/image.{jpg,png,...} __name:=''
python test_subscriber_lag.py image:=/image_raw _buff_size:=<buff_size> _use_message_filters:=<true/false>
```

Warning messages should be displayed indicating how many messages are dropped by the subscriber.
"""

from __future__ import absolute_import, division, print_function

import sys

import rospy
from sensor_msgs.msg import Image

DEFAULT_QUEUE_SIZE = 1
DEFAULT_BUFF_SIZE = 2 ** 16  # [byte]
DEFAULT_DELAY = 1.0  # [seconds]
DEFAULT_IMAGE_TOPIC = 'image'


class TestSubscriberLag:

    def __init__(self):

        # Node setup
        self.node_name = 'test_subscriber_lag'
        rospy.init_node(self.node_name, argv=sys.argv)

        # Params
        self.queue_size = rospy.get_param('~queue_size', default=DEFAULT_QUEUE_SIZE)
        self.buff_size = rospy.get_param('~buff_size', default=DEFAULT_BUFF_SIZE)
        self.delay = rospy.get_param('~delay', default=DEFAULT_DELAY)
        self.use_message_filters = rospy.get_param('~use_message_filters', default=False)

        # Node variables
        self.last_sequence_num = None

        if self.use_message_filters:
            import message_filters
            subscriber = message_filters.Subscriber(DEFAULT_IMAGE_TOPIC, Image,
                                                    buff_size=self.buff_size)
            ts = message_filters.ApproximateTimeSynchronizer([subscriber],
                                                             queue_size=self.queue_size,
                                                             slop=0.1)
            ts.registerCallback(self.image_callback)
        else:
            rospy.Subscriber(DEFAULT_IMAGE_TOPIC, Image,
                             callback=self.image_callback,
                             queue_size=self.queue_size,
                             buff_size=self.buff_size)

    def image_callback(self, image_msg):

        # Simulate image processing
        rospy.sleep(self.delay)

        # Calculate number of dropped messages by subscriber based on sequence numbers
        sequence_num = image_msg.header.seq
        if self.last_sequence_num is None:
            self.last_sequence_num = sequence_num
            return

        rospy.loginfo('Processing image with sequence number: {}'.format(sequence_num))
        num_dropped_msgs = sequence_num - self.last_sequence_num - 1
        if num_dropped_msgs > 0:
            rospy.logwarn('Subscriber dropped messages: {}'.format(num_dropped_msgs))
        self.last_sequence_num = sequence_num


def main():
    TestSubscriberLag()
    rospy.spin()


if __name__ == '__main__':
    main()
