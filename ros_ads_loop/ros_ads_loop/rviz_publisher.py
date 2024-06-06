import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros
from visualization_msgs.msg import Marker
import time

class QuaternionSubscriberNode(Node):
    def __init__(self):
        super().__init__('quaternion_subscriber_node')
        self.subscription = self.create_subscription(
            Quaternion,
            'dynamic_quaternion',
            self.listener_callback,
            10
        )
        self.br = tf2_ros.TransformBroadcaster(self)
        self.model_pub = self.create_publisher(Marker, 'virtual_sat_model', 10)
        self.get_logger().info("Quaternion Subscriber Node has been started")

    def listener_callback(self, msg):
        self.get_logger().info(f'Received quaternion: {msg.x}, {msg.y}, {msg.z}, {msg.w}')

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'virtual_sat'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg

        self.br.sendTransform(t)
        self.publish_model(msg)

    def publish_model(self, quaternion_msg):
        mesh = Marker()
        mesh.header.frame_id = "virtual_sat"
        mesh.header.stamp = self.get_clock().now().to_msg()
        mesh.type = mesh.MESH_RESOURCE
        mesh.mesh_resource = "package://ros_ads_loop/models/VirtualSatv8.stl"
        mesh.pose.orientation = quaternion_msg
        mesh.pose.position.x = 0.0
        mesh.pose.position.y = 0.0
        mesh.pose.position.z = 0.0
        mesh.scale.x = 1.0 / 100  # Adjust scale as needed
        mesh.scale.y = 1.0 / 100
        mesh.scale.z = 1.0 / 100
        mesh.color.a = 1.0  # Alpha must be non-zero
        mesh.color.r = 1.0  # Red color
        mesh.color.g = 0.0
        mesh.color.b = 0.0

        self.model_pub.publish(mesh)

def main(args=None):
    time.sleep(10)
    rclpy.init(args=args)
    node = QuaternionSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
