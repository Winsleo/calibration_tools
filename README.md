# 依赖
- ROS
```bash
wget http://fishros.com/install -O fishros && . fishros
```
- 相机内参标定：camera_calibration包
```bash
sudo apt install ros-${ROS_DISTRO}-camera-calibration
```
- 相机雷达联合标定：image_view2
```bash
sudo apt install ros-${ROS_DISTRO}-image-view2
```
- 相机雷达标定结果测试：jsk_recognition_msgs
```bash
sudo apt install ros-${ROS_DISTRO}-jsk-recognition-msgs
```
# 操作步骤
1. 新建工作空间
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```
2.  clone功能包并且切换到指定分支(opencv4.x环境需要切换到opencv4分支，版本小于opencv4则不用)
```bash
git clone https://github.com/KeepOnKeepOn/manual_dynamic_reconfigure.git
cd manual_dynamic_reconfigure
git checkout opencv4
```
3. 在工作空间catkin_ws中进行编译
```bash
cd ../../
catkin_make 
source /devel/setup.bash
```
## 相机内参标定
### 准备bag数据
bag需要包含相机图像数据，可以是原始图像格式，也可以是压缩图像格式。
图像数据的话题名需要和launch文件中的`image_src`参数相同。
如果是压缩图像格式，需要将`compressed_stream`参数设置为`true`，压缩图像话题为`${image_src}/compressed`形式。
### 启动launch文件
launch文件里的参数bagfile, image_src, size, square, k-coefficients为默认值，用户可根据自身场景修改
- bagfile：bag文件路径
- image_src：原始图像topic
- size: 棋盘格标定板的格子数-1
- square：每个小格的边长，单位m
- k-coefficients: 畸变系数

```bash
roslaunch calibration_bringup camera_calibration.launch playbag:=true compressed_stream:=true
```



相机内参标定结果测试
```bash
roslaunch calibration_bringup display_camera_calibration.launch playbag:=true compressed_stream:=true
```



## 相机雷达联合标定
### 准备bag数据
bag需要包含相机图像数据和雷达点云数据，图像可以是原始图像格式，也可以是压缩图像格式。
图像数据的话题名需要和launch文件中的`image_src`参数相同。
如果是压缩图像格式，需要将`compressed_stream`参数设置为`true`，压缩图像话题为`${image_src}/compressed`形式。
### 启动launch文件
```bash
roslaunch calibration_bringup lidar_camera_calibration.launch playbag:=true compressed_stream:=true
```
在rviz中选择雷达显示雷达点云的话题数据，将rviz坐标系设置为雷达坐标系。




**联合标定结果测试**
```bash
roslaunch calibration_bringup display_lidar_camera_calibration.launch playbag:=true compressed_stream:=true
```






