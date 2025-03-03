# Ros2 Wrapper for ORB_SLAM3

Original repository: [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

## 1. Prerequisites

We have tested the library in **Ubuntu 22.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

### C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

Follow the building instructions and then install the files:
```bash
cd build
sudo make install
```

### OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.

Can be installed via apt on Ubuntu.

### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

Can also be installed via apt on Ubuntu:
```bash
sudo apt install libeigen3-dev
```

### DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

### Ros2

We have tested the code using Ros2 Humble.

## Realsense SDK

Can be installed via apt on ubuntu.

## 3. Build ORB-SLAM3 library and Ros Node

Clone the repository:
```
mkdir -p orb_slam3_ros2_ws/src
cd orb_slam3_ros2_ws/src
git clone xxx
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM3*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ../..
chmod +x build.sh
./build.sh
```

## 4. Running with Intel Realsense D435i

Run the built node using
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run orb_slam3 orb_slam3_node --ros-args -p Vocabulary:=<Your vocabulary> -p Camera_yaml:=<Your configuration file>
```

`<Your vocabulary>` for Realsense D435i can be found under `Vocabulary/ORBvoc.txt`.

`<Your configuration file>` for Realsense D435i can be found under `Examples/Stereo-Intertial/RealSense_D435i.yaml`. See [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) for more details.