from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, '/shelfino0/plan1', 10)
        self.timer_ = self.create_timer(1.0, self.publish_path_msg)

    def publish_path_msg(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        pose1 = PoseStamped()
        pose1.pose.position.x = 0.0
        pose1.pose.position.y = 0.0
        pose2 = PoseStamped()
        pose2.pose.position.x = 2.0
        pose2.pose.position.y = 3.0
        # ...
        path_msg.poses.append(pose1)
        path_msg.poses.append(pose2)
        # ...
        self.publisher_.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

