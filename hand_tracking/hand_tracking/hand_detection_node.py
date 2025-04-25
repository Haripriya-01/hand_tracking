import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

class HandDetectionNode(Node):
    def __init__(self):
        super().__init__('hand_detection_node')
        self.publisher_img = self.create_publisher(Image, 'hand_detected/image', 10)
        self.publisher_coords = self.create_publisher(Float32MultiArray, 'hand_coordinates', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.detect_hand)

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

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_img.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

