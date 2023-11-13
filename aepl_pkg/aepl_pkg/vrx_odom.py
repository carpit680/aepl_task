import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point, Quaternion
import tf2_ros
import math

class OdometryProcessor(Node):
    def __init__(self):
        super().__init__('odometry_processor')
        self.subscription = self.create_subscription(
            Odometry,
            '/wamv/sensors/position/ground_truth_odometry',  # Change this topic to the one where odometry messages are published
            self.odometry_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'odom', 10)

        # Initial values for spawn position and orientation
        self.spawn_position = Point(x=0.0, y=0.0, z=0.0)
        self.first_odom_received = False
        self.spawn_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Create a TransformBroadcaster with the Node instance
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odometry_callback(self, msg):
        # Calculate relative odometry
        if not self.first_odom_received:
            # Set the zero point odom when the first odom reading is received
            self.spawn_position = msg.pose.pose.position
            self.first_odom_received = True

        relative_odom = Odometry()
        relative_odom.header = msg.header
        relative_odom.pose.pose.position.x = msg.pose.pose.position.x - self.spawn_position.x
        relative_odom.pose.pose.position.y = msg.pose.pose.position.y - self.spawn_position.y
        relative_odom.pose.pose.position.z = msg.pose.pose.position.z - self.spawn_position.z

        # Calculate relative orientation (yaw)
        current_orientation = msg.pose.pose.orientation
        spawn_orientation = self.spawn_orientation
        relative_odom.pose.pose.orientation.x = (
            spawn_orientation.w * current_orientation.x +
            spawn_orientation.x * current_orientation.w +
            spawn_orientation.y * current_orientation.z -
            spawn_orientation.z * current_orientation.y
        )
        relative_odom.pose.pose.orientation.y = (
            spawn_orientation.w * current_orientation.y +
            spawn_orientation.y * current_orientation.w +
            spawn_orientation.z * current_orientation.x -
            spawn_orientation.x * current_orientation.z
        )
        relative_odom.pose.pose.orientation.z = (
            spawn_orientation.w * current_orientation.z +
            spawn_orientation.z * current_orientation.w +
            spawn_orientation.x * current_orientation.y -
            spawn_orientation.y * current_orientation.x
        )
        relative_odom.pose.pose.orientation.w = (
            spawn_orientation.w * current_orientation.w -
            spawn_orientation.x * current_orientation.x -
            spawn_orientation.y * current_orientation.y -
            spawn_orientation.z * current_orientation.z
        )

        # Publish relative odometry
        self.publisher.publish(relative_odom)

        # Publish transform between odom and wamv/wamv/base_link frames
        transform_stamped = TransformStamped()
        transform_stamped.header = msg.header
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'wamv/wamv/base_link'
        transform_stamped.transform.translation.x = relative_odom.pose.pose.position.x
        transform_stamped.transform.translation.y = relative_odom.pose.pose.position.y
        transform_stamped.transform.translation.z = relative_odom.pose.pose.position.z
        transform_stamped.transform.rotation = relative_odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)

    odometry_processor = OdometryProcessor()

    rclpy.spin(odometry_processor)

    odometry_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
