import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32, Int32
from cv_bridge import CvBridge

class HandDetectionNode(Node):
    def _init_(self):
        super()._init_('hand_detection_node')
        self.publisher_img = self.create_publisher(Image, 'hand_detected/image', 10)
        self.publisher_coords = self.create_publisher(Float32MultiArray, 'hand_coordinates', 10)
        self.publisher_confidence = self.create_publisher(Float32, 'hand_confidence', 10)
        self.publisher_frame_count = self.create_publisher(Int32, 'frame_count', 10)
        self.publisher_hand_size = self.create_publisher(Float32MultiArray, 'hand_size', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.detect_hand)
        self.frame_count = 0

    def detect_hand(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define skin color range in HSV
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)

        # Mask skin color
        mask = cv2.inRange(hsv, lower_skin, upper_skin)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Publish bounding box coordinates
            msg = Float32MultiArray()
            msg.data = [float(x), float(y), float(w), float(h)]
            self.publisher_coords.publish(msg)

            # Publish confidence score based on contour area
            area = cv2.contourArea(largest_contour)
            confidence_msg = Float32()
            confidence_msg.data = min(area / 10000.0, 1.0)  # normalized 0-1
            self.publisher_confidence.publish(confidence_msg)

            # Publish hand size separately
            size_msg = Float32MultiArray()
            size_msg.data = [float(w), float(h)]
            self.publisher_hand_size.publish(size_msg)

        # Publish processed image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_img.publish(img_msg)

        # Publish frame count
        self.frame_count += 1
        frame_msg = Int32()
        frame_msg.data = self.frame_count
        self.publisher_frame_count.publish(frame_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
