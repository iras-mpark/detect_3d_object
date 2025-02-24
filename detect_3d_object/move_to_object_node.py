import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # ROS 2 parameter declare
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('target_frame', 'chair_frame')

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher set up
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # publisher timer
        self.timer = self.create_timer(1.0, self.publish_goal_pose)

    def publish_goal_pose(self):
        source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        try:
            # get transform between source and target frame
            transform = self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())

            # PoseStamped msg
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = source_frame

            goal_pose.pose.position.x = transform.transform.translation.x
            goal_pose.pose.position.y = transform.transform.translation.y
            goal_pose.pose.position.z = transform.transform.translation.z

            goal_pose.pose.orientation = transform.transform.rotation

            # msg publish
            self.goal_publisher.publish(goal_pose)
            self.get_logger().info(f'Published goal_pose from {source_frame} to {target_frame}: {goal_pose}')

        except tf2_ros.LookupException:
            self.get_logger().warn(f'TF lookup failed: Frame {target_frame} not found from {source_frame}')
        except tf2_ros.ConnectivityException:
            self.get_logger().warn('TF connectivity error')
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn('TF extrapolation error')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

