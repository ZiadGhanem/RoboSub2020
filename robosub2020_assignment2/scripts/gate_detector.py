#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
import heapq
import numpy as np

class GateDetector(Node):

    def __init__(self):
        super().__init__('gate_detector')

        # Orange filter parameters
        self.declare_parameter("orange_min")
        self.declare_parameter("orange_max")
        self.declare_parameter("area_threshold")
        self._orange_min = np.array(self.get_parameter("orange_min").get_parameter_value().integer_array_value, dtype=np.uint8)
        self._orange_max = np.array(self.get_parameter("orange_max").get_parameter_value().integer_array_value, dtype=np.uint8)
        # Contour area threshold parameter
        self._area_threshold = self.get_parameter("area_threshold").get_parameter_value().integer_value


        self._bridge = CvBridge()

        # Video frame subscriber
        self._subscription = self.create_subscription(Image, 'video_stream',
                                                     self.detect_gate, 1)

        # Gate frame publisher
        self._gate_publisher = self.create_publisher(Image, 'gate', 1)
        # Gate location
        self._gate_location_publisher = self.create_publisher(Int32MultiArray, 'gate_location', 1)

    def detect_rectangular_contours(self, frame, area_threshold=300, height_width_ratio=2.5):
        # find the contours from the thresholded image
        _, contours, _ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        result = []
        for contour in contours:
            area = cv2.contourArea(contour)
            # If the contour's area is above a certain threshold
            if area > area_threshold:
                contour_approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
                # If contour is a square or a rectangle
                if (4 <= len(contour_approx) <= 6):
                    # If contour is a vertical rectangle
                    rectangle = cv2.boundingRect(contour)
                    (x, y, width, height) = rectangle
                    if (height / height_width_ratio) > width:
                        result.append([rectangle, area])

        if len(result) > 3:
            result = heapq.nlargest(3, result, key=lambda x: x[1])

        return [rectangle for rectangle, _ in result]

    def get_contour_center(self, contour):
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)

    def get_gate_location(self, frame, rectangles):
        gate_center = None

        # Sort the rectangles from left to right
        rectangles = sorted(rectangles, key=lambda x: x[0])

        if len(rectangles) == 3:
            if (rectangles[1][0] - rectangles[0][0]) < (rectangles[2][0] - rectangles[1][0]):
                small_index = 0
                large_index = 2
            else:
                small_index = 2
                large_index = 0

            red_pts = np.array([[rectangles[1][0], rectangles[1][1]],
                                [rectangles[1][0], rectangles[1][1] + rectangles[large_index][3]],
                                [rectangles[large_index][0], rectangles[large_index][1] + rectangles[large_index][3]],
                                [rectangles[large_index][0], rectangles[large_index][1]]],
                                np.int32)

            green_pts = np.array([[rectangles[1][0], rectangles[1][1]],
                                [rectangles[1][0], rectangles[1][1] + rectangles[small_index][3]],
                                [rectangles[small_index][0], rectangles[small_index][1] + rectangles[small_index][3]],
                                [rectangles[small_index][0], rectangles[small_index][1]]],
                                np.int32)



            red_pts = red_pts.reshape((-1, 1, 2))
            green_pts = green_pts.reshape((-1, 1, 2))

            # Get the small gate center
            gate_center = self.get_contour_center(green_pts)

            # Draw the gate
            frame = cv2.polylines(frame, [red_pts], True, (0, 0, 255), 3)
            frame = cv2.polylines(frame, [green_pts], True, (0, 255, 0), 3)

        # If only two lines were found draw a bouding rectangle
        elif len(rectangles) == 2:
            pts = np.array([[rectangles[0][0], rectangles[0][1]],
                            [rectangles[0][0], rectangles[0][1] + max(rectangles[0][3], rectangles[1][3])],
                            [rectangles[1][0], rectangles[1][1] + max(rectangles[0][3], rectangles[1][3])],
                            [rectangles[1][0], rectangles[1][1]]],
                            np.int32)
            pts = pts.reshape((-1, 1, 2))
            frame = cv2.polylines(frame, [pts], True, (255, 0, 0), 3)

            gate_center = self.get_contour_center(pts)

        # If only one line was found draw a single vertical line
        elif len(rectangles) == 1:
            frame = cv2.line(frame, (rectangles[0][0], 0), (rectangles[0][0], frame.shape[0] - 1), (255, 0, 0), thickness=3)
            gate_center = (int(rectangles[0][0] + rectangles[0][2] / 2),
                           int(rectangles[0][1] + rectangles[0][3] / 2))

        # Draw gate center
        if gate_center is not None:
            frame = cv2.circle(frame, gate_center, 4, (0, 255, 0), -1)

        return frame, gate_center

    def draw_grid_lines(self, frame):
        height, width = frame.shape[:2]

        # Draw vertical lines
        frame = cv2.line(frame, (int(width / 3), 0), (int(width / 3), height - 1), (255, 255, 255), thickness=2)
        frame = cv2.line(frame, (int(2 * width / 3), 0), (int(2 * width / 3), height - 1), (255, 255, 255), thickness=2)

        #Draw horizontal lines
        frame = cv2.line(frame, (0, int(height / 3)), (width - 1, int(height / 3)), (255, 255, 255), thickness=2)
        frame = cv2.line(frame, (0, int(2 * height / 3)), (width - 1, int(2 * height / 3)), (255, 255, 255), thickness=2)

        return frame


    def detect_gate(self, data):
        self.get_logger().info("Received video frame")
        # Read video frame
        try:
            frame = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Make a copy of the original frame
        frame_copy = frame.copy()
        # Change color channel from BGR to HSV
        hsv_frame = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)
        # Apply mask
        thresholded_frame = cv2.inRange(hsv_frame, self._orange_min, self._orange_max)
        # Apply gaussian blur
        thresholded_frame = cv2.GaussianBlur(thresholded_frame, (5, 5), 0)
        # Apply Erosion to remove small noises
        thresholded_frame = cv2.erode(thresholded_frame, np.ones((3, 3), np.uint8), iterations=1)
        # Apply dilation to widen the remaining parts
        thresholded_frame = cv2.dilate(thresholded_frame, np.ones((3, 3), np.uint8), iterations=2)
        # Detect Rectangular contours
        rectangles = self.detect_rectangular_contours(thresholded_frame, self._area_threshold)
        # Draw the gate and get its center
        frame, gate_center = self.get_gate_location(frame, rectangles)
        # Draw the grid lines
        frame = self.draw_grid_lines(frame)

        # Convert frame using CVBridge
        frame = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # Publish the gate frame
        self._gate_publisher.publish(frame)
        # Publish the gate location
        if gate_center is not None:
            gate_center_msg = Int32MultiArray()
            gate_center_msg.data = list(gate_center)
            self._gate_location_publisher.publish(gate_center_msg)


def main(args=None):
    rclpy.init(args=args)

    gate_detector = GateDetector()

    rclpy.spin(gate_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gate_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
