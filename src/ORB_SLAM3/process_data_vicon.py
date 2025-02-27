import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import PoseStamped  # 根据实际消息类型修改

def pose_to_matrix(x, y, z, qx, qy, qz, qw):
    rotation = R.from_quat([qx, qy, qz, qw]).as_matrix()
    translation = np.array([x, y, z])
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T

# 读取数据
test_data = np.loadtxt("FrameTrajectory_TUM_Format_3.txt", delimiter=" ")
test_time = test_data[:, 0]
test_pos = test_data[:, 1:4]

test_time_start = test_data[0, 0]
test_time_stop = test_data[-1, 0]
test_pos_list_val = np.array([])
test_pos_start = test_data[0, 1:4]

bag_path = "../data/vicon3.bag"

# 配置存储和转换选项
storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
converter_options = ConverterOptions(
    input_serialization_format="cdr", output_serialization_format="cdr"
)

# 创建读取器
reader = SequentialReader()
reader.open(storage_options, converter_options)

# 遍历所有消息
gt_data = np.empty([0, 8])
gt_time_list = np.array([])
gt_dist_list = np.array([])
flag = 0
while reader.has_next():
    (topic, data, timestamp) = reader.read_next()

    # 仅处理目标主题
    if topic == "/vrpn/trailer/pose":
        msg = deserialize_message(data, PoseStamped)
        # print(msg.header.stamp.sec, test_time_start)
        if timestamp / 1e9 >= test_time_start and timestamp / 1e9 <= test_time_stop:
            if flag == 0:
                flag = 1
                start_pos_x = msg.pose.position.x
                start_pos_y = msg.pose.position.y
                start_pos_z = msg.pose.position.z
            gt_dist_list = np.append(gt_dist_list, np.sqrt((msg.pose.position.x - start_pos_x) ** 2 + (msg.pose.position.y - start_pos_y) ** 2 + (msg.pose.position.z - start_pos_z) ** 2))
            # gt_dist_list = np.append(gt_dist_list, msg.pose.position.z)
            gt_time_list = np.append(gt_time_list, timestamp / 1e9)
            gt_data = np.vstack([gt_data, np.array([timestamp / 1e9, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])])

    elif timestamp / 1e9 > test_time_stop:
            break

# 关闭读取器
del reader

test_time = np.array([])
test_data_val = np.empty([0, 8])
flag = 0
for i in range(test_data.shape[0]):
    if test_data[i, 0] >= gt_time_list[0]:
        if flag == 0:
             start_pos_slam = test_data[i, 1:4]
             flag = 1
        dist = np.sqrt(np.sum((start_pos_slam - test_data[i, 1:4]) ** 2))
        test_pos_list_val = np.append(test_pos_list_val, dist)
        test_time = np.append(test_time, test_data[i, 0])
        test_data_val = np.vstack([test_data_val, test_data[i, :]])


import matplotlib.pyplot as plt

plt.plot(test_time, test_pos_list_val, label="slam")
plt.plot(gt_time_list, gt_dist_list, label="vicon")
plt.legend()
plt.savefig('Dist_compare.png')

################ Transform test data to gt coordinates #################
T_test_initial = pose_to_matrix(*test_data_val[0, 1:])
T_gt_initial = pose_to_matrix(*gt_data[0, 1:])
T_test_to_gt = T_test_initial @ np.linalg.inv(T_gt_initial)

transformed_test_data = np.empty([0, 8])
for i in range(test_data_val.shape[0]):
    t, x, y, z, qx, qy, qz, qw = test_data_val[i, :]
    T_test_i = pose_to_matrix(x, y, z, qx, qy, qz, qw)
    T_transformed_i = T_test_to_gt @ T_test_i
    translation = T_transformed_i[:3, 3]
    rotation = R.from_matrix(T_transformed_i[:3, :3]).as_quat()
    transformed_test_data = np.vstack([transformed_test_data, np.array([t, *translation, *rotation])])

plt.clf()
plt.plot(gt_data[:, 0], gt_data[:, 1], label='gt')
plt.plot(transformed_test_data[:, 0], transformed_test_data[:, 1], label='test')
plt.legend()
plt.savefig('test.png')