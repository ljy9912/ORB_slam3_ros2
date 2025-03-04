import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from calibration_msg.msg import CalibrationResult

class CalibPoseNode(Node):
    def __init__(self, wrist_offset=0.05):
        super().__init__('calib_pose_node')
        
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
        self.pubWrist = self.create_publisher(
            Pose,
            '/wrist_pose',
            10
        )
        self.ps = np.zeros(3)
        self.rotation = R.from_quat([0., 0., 0., 1.])
        self.wrist_offset = wrist_offset
        
    def pose_callback(self, msg):
        # 提取位置 (pc) 和四元数 (q)
        self.msg = msg
        self.position_cal = np.dot(self.rotation.as_matrix(), (np.array([msg.position.x, msg.position.y, msg.position.z]) - self.ps))
        self.orientation_cal = (self.rotation * R.from_quat(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))) * R.from_matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        self.orientation_cal = self.orientation_cal.as_quat()
        pose_msg = Pose()
        pose_msg.position.x = self.position_cal[0]
        pose_msg.position.y = self.position_cal[1]
        pose_msg.position.z = self.position_cal[2]
        pose_msg.orientation.x = self.orientation_cal[0]
        pose_msg.orientation.y = self.orientation_cal[1]
        pose_msg.orientation.z = self.orientation_cal[2]
        pose_msg.orientation.w = self.orientation_cal[3]
        self.pubPose.publish(pose_msg)

        self.compute_wrist_pose()
        self.get_logger().info('收到Pose，发送已校准的pose.')

    def compute_wrist_pose(self):
        pww = -self.msg.orientation * self.wrist_offset + self.position
        psw = np.dot(self.rotation.as_matrix(), (pww + self.ps))
        pose_msg = Pose()
        pose_msg.position.x = psw[0]
        pose_msg.position.y = psw[1]
        pose_msg.position.z = psw[2]
        pose_msg.orientation.x = self.orientation_cal[0]
        pose_msg.orientation.y = self.orientation_cal[1]
        pose_msg.orientation.z = self.orientation_cal[2]
        pose_msg.orientation.w = self.orientation_cal[3]
        self.pubWrist.publish(pose_msg)

    def calib_callback(self, msg):
        ps = msg.ps
        self.ps = np.array([ps.x, ps.y, ps.z])
        self.rotation = R.from_quat(np.array([msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]))

        # self.get_logger().info('收到标定结果，更新校准参数.')

def main(args=None):
    rclpy.init(args=args)
    node = CalibPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()