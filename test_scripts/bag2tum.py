# 保存为 extract_pose.py
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import csv
from geometry_msgs.msg import Pose
import sys

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 extract_pose.py input.db3 output.csv")
        return

    bag_path = sys.argv[1]
    output_csv = sys.argv[2]

    # 初始化ROS2 Python接口
    rclpy.init()
    
    # 打开rosbag
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # 创建CSV文件
    with open(output_csv, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])

        # 遍历所有消息
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic == '/camera_pose':  # 替换为你的轨迹话题
                msg = deserialize_message(data, Pose)
                pose = msg
                writer.writerow([
                    t, 
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, 
                    pose.orientation.z, pose.orientation.w
                ])

    # reader.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()