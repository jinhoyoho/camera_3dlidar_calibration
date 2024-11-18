# Camera 3D LiDAR Calibration

## Introduction

Web camera and 3D Lidar calibration code
(https://velog.io/@jinhoyoho/3D-lidar-camera-calibration)

**You have to change intrinsic and extrinsic parameter with my code!**

![rqt_graph](./jpg/rqt_graph.png)

## Installation

### Prerequisites

- Ubuntu = 20.04
- ROS1 Noetic
- Python
- Pytorch
- CUDA
- Ultralytics

## Clone this repository

```Shell
git clone https://github.com/jinhoyoho/camera_3dlidar_calibration.git
```

## Create a conda virtual environment and activate it (conda is optional)

```Shell
# Create conda virtual environment
conda create -n calibration python=3.12 -y
# Activate conda virtual environment
conda activate calibration

# Install requirements
pip install -r requirements.txt

# Install Pytorch and CUDA
pip

```

## Running

```Shell
cd ~/catkin_ws/
catkin_make
```

```Shell
rosrun camera_3dlidar_calibration camera.py
rosrun camera_3dlidar_calibration calibration
```

## Result

![result1](./jpg/result1.png)

![result1](./jpg/result2.png)

![result1](./jpg/result3.png)
