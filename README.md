# yolonas_ros

ROS 2 wrap for [YOLO-NAS](https://github.com/Deci-AI/super-gradients/blob/master/YOLONAS.md) to perform object detection and tracking.

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/yolonas_ros.git
$ pip3 install -r yolonas_ros/requirements.txt
$ cd ~/ros2_ws
$ colcon build
```

## Usage

```shell
$ ros2 launch yolonas_bringup yolonas.launch.py
```
