# Use ros2 commands
source /opt/ros/humble/setup.bash
source ../colcon_ws/install/setup.bash
ros2 run orbslam3 stereo-inertial Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml false

ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true enable_gyro:=true enable_accel:=true infra_width:=640 infra_height:=480 enable_sync:=true unite_imu_method:=2