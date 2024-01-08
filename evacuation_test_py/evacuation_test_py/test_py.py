import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

class ObstaclesSubscriber(Node):
    def __init__(self):
        super().__init__('obstacles_subscriber')
        self.subscription = self.create_subscription(
            String,
            'obstacles',
            self.obstacles_callback,
            10)
        self.subscription  # 防止被垃圾回收

    def obstacles_callback(self, msg):
        # 在这里处理接收到的消息
        self.get_logger().info('yes')
        self.get_logger().info(f'Received obstacles message:\n{msg}')

def main(args=None):
    rclpy.init(args=args)
    obstacles_subscriber = ObstaclesSubscriber()
    rclpy.spin(obstacles_subscriber)
    obstacles_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()