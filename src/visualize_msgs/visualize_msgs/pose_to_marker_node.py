import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

class PoseToMarkerNode(Node):
    def __init__(self):
        super().__init__('pose_to_marker_node')
        # 订阅 Pose 话题
        self.subscription = self.create_subscription(
            Pose, '/camera_pose_calibrated', self.pose_callback, 10)
        self.subscription_raw = self.create_subscription(
            Pose, '/camera_pose', self.pose_raw_callback, 10
        )
        # 发布 Marker 话题
        self.marker_pub = self.create_publisher(Marker, '/camera_pose_calibrated_marker', 10)
        self.marker_raw_pub = self.create_publisher(Marker, '/camera_pose_marker', 10)


    def pose_callback(self, msg):
        # 创建 Marker 消息
        marker = Marker()
        marker.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        marker.ns = 'pose_marker'
        marker.id = 0
        marker.type = Marker.ARROW  # 使用箭头表示 Pose
        marker.action = Marker.ADD
        marker.pose = msg  # 直接使用 Pose 消息
        marker.scale.x = 0.5  # 箭头长度
        marker.scale.y = 0.1  # 箭头宽度
        marker.scale.z = 0.1  # 箭头高度
        marker.color.a = 1.0  # 透明度
        marker.color.r = 0.0  # 红色
        marker.color.g = 1.0  # 绿色
        marker.color.b = 0.0  # 蓝色

        # 发布 Marker
        self.marker_pub.publish(marker)

    def pose_raw_callback(self, msg):
        # 创建 Marker 消息
        marker = Marker()
        marker.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        marker.ns = 'pose_raw_marker'
        marker.id = 0
        marker.type = Marker.ARROW  # 使用箭头表示 Pose
        marker.action = Marker.ADD
        marker.pose = msg  # 直接使用 Pose 消息
        marker.scale.x = 0.5  # 箭头长度
        marker.scale.y = 0.1  # 箭头宽度
        marker.scale.z = 0.1  # 箭头高度
        marker.color.a = 1.0  # 透明度
        marker.color.r = 0.0  # 红色
        marker.color.g = 0.0  # 绿色
        marker.color.b = 1.0  # 蓝色

        # 发布 Marker
        self.marker_raw_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToMarkerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()