import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'hand_coordinates', self.track_hand, 10)
        self.marker_publisher = self.create_publisher(Marker, 'hand_marker', 10)

    def track_hand(self, msg):
        if len(msg.data) != 4:
            return
        
        x, y, w, h = msg.data
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = w / 100.0
        marker.scale.y = h / 100.0
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position.x = x / 100.0
        marker.pose.position.y = y / 100.0
        marker.pose.position.z = 0.0

        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

