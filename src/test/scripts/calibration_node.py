import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from calibration_msg.msg import CalibrationResult

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        # 订阅相机位姿
        self.subscription = self.create_subscription(
            Pose,
            '/camera_pose',
            self.pose_callback,
            10
        )
        
        # 发布标定结果
        self.publisher = self.create_publisher(
            CalibrationResult,
            '/calibration_result',
            10
        )
        
        # 初始化数据存储
        self.A_rows = []
        self.b_rows = []

    def pose_callback(self, msg):
        # 提取位置 (pc) 和四元数 (q)
        pc = np.array([msg.position.x, msg.position.y, msg.position.z])
        q = np.array([msg.orientation.x, msg.orientation.y, 
                      msg.orientation.z, msg.orientation.w])
        
        try:
            # 计算旋转矩阵
            Rc = R.from_quat(q).as_matrix()
        except Exception as e:
            self.get_logger().error(f"四元数转换错误: {e}")
            return
        
        # 构建矩阵块
        block = np.hstack([np.eye(3), -Rc])
        self.A_rows.append(block)
        self.b_rows.append(pc.reshape(-1, 1))
        
        # 构造最小二乘问题
        A = np.vstack(self.A_rows)
        b = np.vstack(self.b_rows)
        
        try:
            x, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"矩阵求解错误: {e}")
            return
        
        # 提取结果
        ps = x[:3].flatten()
        offset = x[3:].flatten()
        residual = residuals[0] if residuals.size > 0 else 0.0
        
        # 发布结果
        result_msg = CalibrationResult()
        result_msg.ps.x, result_msg.ps.y, result_msg.ps.z = ps
        result_msg.offset.x, result_msg.offset.y, result_msg.offset.z = offset
        result_msg.residual = residual
        self.publisher.publish(result_msg)
        self.get_logger().info('发布标定结果')

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()