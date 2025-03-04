import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from calibration_msg.msg import CalibrationResult

class CalibPoseNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        # 订阅相机位姿
        self.subCam = self.create_subscription(
            Pose,
            '/camera_pose',
            self.pose_callback,
            10
        )
        self.subCalib = self.create_subscription(
            CalibrationResult,
            '/calibration_result',
            self.calib_callback,
            10
        )
        self.pubPose = self.create_publisher(
            Pose,
            '/camera_pose_calibrated',
            10
        )
        self.ps = np.zeros(3)
        self.rotation = R.from_quat([0., 0., 0., 1.])
        
    def pose_callback(self, msg):
        # 提取位置 (pc) 和四元数 (q)
        self.msg = msg
        position = np.dot(self.rotation.as_matrix(), (np.array([msg.position.x, msg.position.y, msg.position.z]) - self.ps))
        orientation = (self.rotation * R.from_quat(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))).as_quat()
        pose_msg = Pose()
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        pose_msg.orientation.x = orientation[0]
        pose_msg.orientation.y = orientation[1]
        pose_msg.orientation.z = orientation[2]
        pose_msg.orientation.w = orientation[3]
        self.pubPose.publish(pose_msg)

        self.get_logger().info('收到Pose，发送已校准的pose.')

    def calib_callback(self, msg):
        ps = msg.ps
        self.ps = np.array([ps.x, ps.y, ps.z])
        self.rotation = R.from_quat(np.array([msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]))

        self.get_logger().info('收到标定结果，更新校准参数.')

def main(args=None):
    rclpy.init(args=args)
    node = CalibPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()