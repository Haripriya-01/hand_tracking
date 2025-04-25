import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time

class HandTrackingNode(Node):
    def _init_(self):
        super()._init_('hand_tracking_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'hand_coordinates', self.track_hand, 10)
        self.marker_publisher = self.create_publisher(Marker, 'hand_marker', 10)
        self.center_marker_publisher = self.create_publisher(Marker, 'hand_center_marker', 10)

    def track_hand(self, msg):
        if len(msg.data) != 4:
            return
        
        x, y, w, h = msg.data

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
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

        # Publish hand center marker
        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.scale.x = 0.05
        center_marker.scale.y = 0.05
        center_marker.scale.z = 0.05
        center_marker.color.a = 1.0
        center_marker.color.r = 0.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0
        center_marker.pose.position.x = (x + w/2) / 100.0
        center_marker.pose.position.y = (y + h/2) / 100.0
        center_marker.pose.position.z = 0.0

        self.center_marker_publisher.publish(center_marker)

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
