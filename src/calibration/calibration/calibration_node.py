import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from calibration_msg.msg import CalibrationResult
from calibration.RecursiveLS import RecursiveLeastSquares
from enum import Enum

class CalibState(Enum):
    INIT = 1          # 初始状态，等待最小二乘收敛
    PROMPT = 2        # 提示用户进入标定
    STABLE_CHECK = 3  # 检测位姿稳定性
    CALIBRATED = 4        # 记录数据并计算旋转矩阵

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
        self.get_logger().info('开始标定... 请保持手臂伸直并持续移动。')
        
        # 发布标定结果
        self.publisher = self.create_publisher(
            CalibrationResult,
            '/calibration_result',
            10
        )
        
        # 初始化数据存储
        self.A_rows = []
        self.b_rows = []
        self.points_num = 0
        self.ls_residual = 100
        self.rls = RecursiveLeastSquares(delta=1)
        self.state = CalibState.INIT
        self.timer = self.create_timer(0.1, self.state_machine)  # 状态机周期执行
        self.rotation = np.array([0., 0., 0., 1.])

    def state_machine(self):
        if self.state == CalibState.INIT:
            if self.least_square_converged():  # 最小二乘收敛判断
                self.get_logger().info('最小二乘收敛，请保持手臂伸直并指向正前方。')
                self.state = CalibState.PROMPT

        elif self.state == CalibState.PROMPT:
            # 此处可添加倒计时或等待用户输入确认（如需）
            self.start_stable_check()
            self.state = CalibState.STABLE_CHECK

        elif self.state == CalibState.STABLE_CHECK:
            if self.check_pose_stable(threshold=1e-2):  # 持续检测位姿稳定性
                self.calibrate()
                self.state = CalibState.CALIBRATED
                self.get_logger().info('标定完成！')

        elif self.state == CalibState.CALIBRATED:
            pass

    def pose_callback(self, msg):
        # 提取位置 (pc) 和四元数 (q)
        if self.state == CalibState.INIT:
            self.points_num += 1
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
            A_c = block
            b_c = pc.reshape(-1, 1)
            self.A_rows.append(block)
            self.b_rows.append(pc.reshape(-1, 1))
        
            # 构造最小二乘问题
            A = np.vstack(self.A_rows)
            b = np.vstack(self.b_rows)
        
            try:
                x, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)
                # x, residuals, _, _ = self.rls.solve(A_c, b_c)
            except np.linalg.LinAlgError as e:
                self.get_logger().error(f"矩阵求解错误: {e}")
                return
        
            # 提取结果
            self.ps = x[:3].flatten()
            self.offset = x[3:].flatten()
            self.residual = residuals[0] / (A.shape[0] * 3) if residuals.size > 0 else 0.0
        
        # 发布结果
        result_msg = CalibrationResult()
        result_msg.ps.x, result_msg.ps.y, result_msg.ps.z = self.ps
        result_msg.offset.x, result_msg.offset.y, result_msg.offset.z = self.offset
        result_msg.residual = self.residual
        result_msg.rotation.x = self.rotation[0]
        result_msg.rotation.y = self.rotation[1]
        result_msg.rotation.z = self.rotation[2]
        result_msg.rotation.w = self.rotation[3]
        self.ls_residual = self.residual
        # print('Residual', self.residual)
        self.publisher.publish(result_msg)
        self.msg = msg

    def least_square_converged(self):
        # 判断最小二乘是否收敛
        if self.points_num > 500:
            return True
        return False

    def start_stable_check(self):
        self.x_list = []
        self.y_list = []
        self.z_list = []

    def check_pose_stable(self, threshold=1e-3):
        # 检测位姿稳定性
        self.check_pose_length = 20
        if len(self.x_list) < self.check_pose_length:
            self.x_list.append(self.msg.position.x)
            self.y_list.append(self.msg.position.y)
            self.z_list.append(self.msg.position.z)
            return False
        else:
            self.x_list = self.x_list[-self.check_pose_length + 1:]
            self.x_list.append(self.msg.position.x)
            self.y_list = self.y_list[-self.check_pose_length + 1:]
            self.y_list.append(self.msg.position.y)
            self.z_list = self.z_list[-self.check_pose_length + 1:]
            self.z_list.append(self.msg.position.z)
            if np.cov(np.array(self.x_list)) < threshold and np.cov(np.array(self.y_list)) < threshold and np.cov(np.array(self.z_list)) < threshold:
                return True
            else:
                return False

    def calibrate(self):
        v_x, v_y = np.mean(self.x_list) - self.ps[0], np.mean(self.y_list) - self.ps[1]
        theta = np.arctan2(v_y, v_x)  # 计算向量与 x 轴的夹角
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
    
        # 构建旋转矩阵
        self.rotation = R.from_matrix(np.array([[cos_theta, sin_theta, 0],
                            [-sin_theta, cos_theta, 0], [0, 0, 1]])).as_quat()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()