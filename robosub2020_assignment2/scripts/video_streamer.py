#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os
import glob

class VideoStreamer(Node):

    def __init__(self):
        super().__init__('video_streamer')
        self._publisher = self.create_publisher(Image, 'video_stream', 1)
        self._bridge = CvBridge()
        self.declare_parameter("dataset_path")
        self.declare_parameter("frame_shape")

        self.publish_frame()

    def resize_frame(self, frame, new_width=None, new_height=None):
        height, width = frame.shape[:2]
        if new_width is None and new_height is None:
            return frame
        if new_width is not None and new_height is None:
            r = new_width/width
            new_height = int(height * r)
        elif new_width is None and new_height is not None:
            r = new_height/height
            new_width = int(width * r)
        new_frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)
        return new_frame

    def publish_frame(self):
        # Get the dataset path parameter
        dataset_path = self.get_parameter("dataset_path").get_parameter_value().string_value
        # Get the resized frame shape
        frame_shape = self.get_parameter("frame_shape").get_parameter_value().integer_array_value

        while True:
            for video_path in glob.glob(os.path.join(dataset_path, "*.mp4")):
                # Read video file
                cap = cv2.VideoCapture(video_path)
                cap.set(cv2.CAP_PROP_POS_FRAMES, 22)

                while cap.isOpened():
                    # Capture frame-by-frame
                    ret, frame = cap.read()
                    if not ret:
                        break

                    # Resize frame
                    frame = self.resize_frame(frame, frame_shape[0])

                    # Convert frame using CVBridge
                    frame = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    # Publish the frame
                    self._publisher.publish(frame)
                    self.get_logger().info("Publishing video frame")

                    cv2.waitKey(20)


def main(args=None):
    rclpy.init(args=args)

    video_streamer = VideoStreamer()

    rclpy.spin(video_streamer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_streamer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
