# zed_open_capture_ros
ROS1 wrapper for zed_open_capture for those who hopes to use zed camera on CPU only machine.

## Installation
1. Install [zed-open-capture library](https://github.com/stereolabs/zed-open-capture)
2. ```bash
    cd catkin_ws/src
    git clone https://github.com/jarvisyjw/zed_open_capture_ros.git
    cd ..
    catkin build
    source devel/setup.bash
    ```

## Usage
```bash
rosrun zed_open_capture_ros stereo_node # stereo cam only
rosrun zed_open_capture_ros stereo_imu_node # stereo cam with imu 
```