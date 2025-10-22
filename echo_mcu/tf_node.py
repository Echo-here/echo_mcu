#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion

class TFNode(Node):
    def __init__(self):
        super().__init__('tf_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
         # TF (odom → base_link)
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        # ✅ TF (base_link → laser)
        tf2 = TransformStamped()
        tf2.header.stamp = msg.header.stamp
        tf2.header.frame_id = 'base_link'
        tf2.child_frame_id = 'laser'
        tf2.transform.translation.x = 0.0
        tf2.transform.translation.y = 0.0
        tf2.transform.translation.z = 0.2  # 약간 띄운 높이
        tf2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf2)

def main(args=None):
    rclpy.init(args=args)
    node = TFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
