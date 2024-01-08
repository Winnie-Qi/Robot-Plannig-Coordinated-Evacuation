import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray, Polygon, PolygonStamped, PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from obstacles_msgs.msg import ObstacleArrayMsg
from std_msgs.msg import Header
import copy

# 假设hycbs_main.py在同一目录下，且有一个可调用的main函数
from .hycbs_main import main as hycbs_main

class EvacuationNode(Node):
    def __init__(self):
        super().__init__('evacuation_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_borders_topic', '/map_borders'),
                ('gate_position_topic', '/gate_position'),
                ('borders_topic', '/obstacles'),
                ('amcl_pose_topics', ['/shelfino0/amcl_pose', '/shelfino1/amcl_pose', '/shelfino2/amcl_pose']),
                ('follow_path_topics', ['/shelfino0/follow_path', '/shelfino1/follow_path', '/shelfino2/follow_path'])
            ]
        )

        # 获取参数
        map_borders_topic = self.get_parameter('map_borders_topic').get_parameter_value().string_value
        gate_position_topic = self.get_parameter('gate_position_topic').get_parameter_value().string_value
        borders_topic = self.get_parameter('borders_topic').get_parameter_value().string_value
        amcl_pose_topics = self.get_parameter('amcl_pose_topics').get_parameter_value().string_array_value
        follow_path_topics = self.get_parameter('follow_path_topics').get_parameter_value().string_array_value

        # self.test_sub = self.create_subscription(ObstacleArrayMsg, borders_topic, self.test_callback, 10)

        # 订阅话题
        self.map_borders_sub = self.create_subscription(Polygon, map_borders_topic, self.map_borders_callback, 10)
        self.gate_position_sub = self.create_subscription(PoseArray, gate_position_topic, self.gate_position_callback, 10)
        self.borders_sub = self.create_subscription(ObstacleArrayMsg, borders_topic, self.borders_callback, 10)
        self.amcl_pose1_subs = self.create_subscription(PoseWithCovarianceStamped, amcl_pose_topics[0], self.amcl_pose1_callback, 10)
        self.amcl_pose2_subs = self.create_subscription(PoseWithCovarianceStamped, amcl_pose_topics[1], self.amcl_pose2_callback, 10)
        self.amcl_pose3_subs = self.create_subscription(PoseWithCovarianceStamped, amcl_pose_topics[2], self.amcl_pose3_callback, 10)


        # 动作客户端
        self.follow_path_clients = [ActionClient(self, FollowPath, topic) for topic in follow_path_topics]

        # 存储接收到的数据
        self.map_borders = None
        self.gate_position = None
        self.obstacles = None
        self.amcl_poses = [None] * len(amcl_pose_topics)

    def map_borders_callback(self, msg):
        points = msg.points
        self.map_borders = [[point.x, point.y] for point in points]
        self.evacuation()
        print("map_borders_callback")

    def test_callback(self, msg):
        print("test")

    def gate_position_callback(self, msg):
        first_pose = msg.poses[0]
        self.gate_position = [first_pose.position.x, first_pose.position.y]
        self.evacuation()
        print("gate_position_callback")

    def borders_callback(self, msg):
        self.obstacles = []
        print("borders_callback start")
        polygon = []
        # 遍历消息中的每个障碍物
        for obstacle in msg.obstacles:
            polygon = []
            for point in obstacle.polygon.points:
                # 提取障碍物的 x 和 y 坐标
                x_coord = point.x
                y_coord = point.y
                polygon.append([x_coord, y_coord])
            self.obstacles.append(polygon)
        print(self.obstacles)
        self.evacuation()
        print("borders_callback")

    def amcl_pose1_callback(self, msg):
        self.amcl_poses[0] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w] 
        self.evacuation()
        print("amcl_pose1_callback")

    def amcl_pose2_callback(self, msg):
        self.amcl_poses[1] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w] 
        self.evacuation()
        print("amcl_pose2_callback")
    
    def amcl_pose3_callback(self, msg):
        self.amcl_poses[2] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w] 
        self.evacuation()
        print("amcl_pose3_callback")

    def evacuation(self):
        print("evacuation")
        if not all([self.map_borders, self.gate_position, self.obstacles, all(self.amcl_poses)]):
            return

        print("all")
        result = hycbs_main(self.amcl_poses, self.map_borders, self.gate_position, self.obstacles)

        print("start")
        adjusted_result = []
        for key in result:
            adjusted_path = copy.deepcopy(result[key])
            adjusted_path = [[x-8 for x in sublist] for sublist in adjusted_path] 
            adjusted_result.append(adjusted_path)

        # 发送结果到FollowPath动作服务器并等待结果
        futures = []
        for i, client in enumerate(self.follow_path_clients):
            follow_path_goal = FollowPath.Goal()
            proper_path_object = self.convert_to_proper_path_type(adjusted_result[i])
            follow_path_goal.path = proper_path_object
            future = client.send_goal_async(follow_path_goal)
            futures.append(future)

        print("All Follow Path Goals Sent and Completed")
        rclpy.shutdown()
    
    def convert_to_proper_path_type(self, adjusted_path):
        # 创建 Path 消息
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rclpy.time.Time().to_msg()  # 设置当前时间
        path_msg.header.frame_id = "map"  # 根据您的需求设置参考框架

        # 遍历调整后的路径点
        for point in adjusted_path:
            # 为每个点创建 PoseStamped 消息
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0 

            # 将 PoseStamped 添加到路径中
            path_msg.poses.append(pose_stamped)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = EvacuationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()