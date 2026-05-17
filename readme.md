# Readme
中文（详细）说明：https://www.jianshu.com/p/4b9c0f4658b1?v=1713596074817
### Intro
This repo provides simple control plan over robotic football, each team consists of several Yanshees and RoboMasters (RM). 

Positional feedback comes from motion capture system.
### Prerequisites
Ubuntu20.

Install ros2-galactic.

Install [robomaster-ros](https://github.com/jeguzzi/robomaster_ros).

Install [RVO2 v2.0.2](https://gamma.cs.unc.edu/RVO2/downloads/).

Use system python to install all required modules, conda could cause unexpected problem like "module not found". 

Install [nokov](https://cloud.tsinghua.edu.cn/d/222902bb277b4f09afaa/) by pip.

Install other python modules:

```
pip install interval sympy
```
Could need to install "empy, lark" by pip too.

For yanshee control, usually need to install nest-asyncio and google protobuf==3.20.0.

```
pip install nest-asyncio protobuf==3.20.0
```

For yanshee API, refer to [this website](https://yandev.ubtrobot.com/#/zh/api?api=YanAPI).

For robomaster API, check [this website](https://robomaster-dev.readthedocs.io/zh-cn/latest/). Better way is to check the [examples](https://github.com/dji-sdk/RoboMaster-SDK/tree/master/examples)
### Build
Pack this repo to a folder 'src', and put it in a workspace.

In workspace, run ```colcon build```.
### Usage
Start XINGYING (Nokov mocap software), check 'SDK' in settings and start running.

If XINGYING starts on another PC, then connect server to the switch of mocap by ethernet cable.(Important note: must be connected AFTER XINGYING is started.)

Run following command on server.
```
ros2 run motion_capture motion_capture
```
If it shows 'motion capture initialization finished!!' then run following commands in new terminals.
```python
ros2 run roscpp pub_pos # obastacle avoidance
ros2 launch rmcontrol ep_startup.launch.py # connect to RM EPs
ros2 launch rmcontrol s1_startup.launch.py # connect to RM S1s
ros2 launch yscontrol ys_control.launch.py # control Yanshees
ros2 run rmcontrol strategy # start control plans for RMs
ros2 launch rmcontrol ep_control.launch.py # control RMs
ros2 run rmcontrol draw_plot # optional. Show current and expected position of all robots and ball
ros2 run motion_capture inbetween #optional. If need a second server in local network to cooperate, then run this 'inbetween' node before ethernet connection to transmit positional info.
```
To stop RMs, stop strategy node and ep_startup.launch.py and s1_startup.launch.py.


### Hardware
./motion_capture: Nokov motion capture system

./rmcontrol: DJI Robomaster EP/S1

./yscontrol: Yanshee from UBTech (Always fall over. Recalibration works badly.)
### About
Provided by Haier Zhu from THU SIGS under instruction of Prof. Li, Xiang.

This work was supported in part by the National Natural Science Foundation of China under Grant U21A20517 and 52075290, and in part by the Science and Technology Innovation 2030-Key Project under Grant 2021ZD0201404.


# 新增：基于夹爪摄像头的视觉抓取成功判定

## 新增功能

本次新增 ROS2 包 `rm_grip_vision`，用于通过 RoboMaster EP 夹爪摄像头判断 RM1、RM2、RM3 是否成功夹取灰色反光网球。

订阅 EP 摄像头图像：

```bash
/RM1/camera/image_color
/RM2/camera/image_color
/RM3/camera/image_color
```

发布抓取判断结果：

```bash
/RM1/grip_vision/caught
/RM2/grip_vision/caught
/RM3/grip_vision/caught
```

可选发布调试图像：

```bash
/RM1/grip_vision/debug_image
/RM2/grip_vision/debug_image
/RM3/grip_vision/debug_image
```

## 新增依赖

```bash
sudo apt install ros-galactic-cv-bridge python3-opencv python3-numpy
```

如果不是 ROS2 Galactic，请将 `galactic` 替换为当前 ROS2 版本。

## 编译命令

```bash
cd ~/ros2_ws
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只重新编译新增视觉包和 RM 控制包：

```bash
cd ~/ros2_ws
source /opt/ros/galactic/setup.bash
colcon build --symlink-install --packages-select rm_grip_vision rmcontrol
source install/setup.bash
```

## 推荐启动命令

先按原流程启动动捕、避障和机器人连接：

```bash
ros2 run motion_capture motion_capture
ros2 run roscpp pub_pos
ros2 launch rmcontrol ep_startup.launch.py
ros2 launch rmcontrol s1_startup.launch.py
ros2 launch yscontrol ys_control.launch.py
```

再启动视觉检测和策略节点：

```bash
ros2 launch rm_grip_vision rm123_grip_vision_and_strategy.launch.py publish_debug:=true
```

注意：该 launch 会同时启动 `rmcontrol strategy`，不要再额外执行：

```bash
ros2 run rmcontrol strategy
```

最后启动 RM 控制节点：

```bash
ros2 launch rmcontrol ep_control.launch.py
```

完整推荐流程：

```bash
ros2 run motion_capture motion_capture
ros2 run roscpp pub_pos
ros2 launch rmcontrol ep_startup.launch.py
ros2 launch rmcontrol s1_startup.launch.py
ros2 launch yscontrol ys_control.launch.py
ros2 launch rm_grip_vision rm123_grip_vision_and_strategy.launch.py publish_debug:=true
ros2 launch rmcontrol ep_control.launch.py
```

## 分开启动视觉检测和策略节点

```bash
ros2 launch rm_grip_vision rm123_grip_vision.launch.py publish_debug:=true
ros2 run rmcontrol strategy --ros-args -p use_visual_catch:=true
ros2 launch rmcontrol ep_control.launch.py
```

## 关闭视觉判断

回退到原动捕判断：

```bash
ros2 run rmcontrol strategy --ros-args -p use_visual_catch:=false
```

## 查看检测结果

查看抓取成功判断：

```bash
ros2 topic echo /RM1/grip_vision/caught
ros2 topic echo /RM2/grip_vision/caught
ros2 topic echo /RM3/grip_vision/caught
```

查看检测置信度：

```bash
ros2 topic echo /RM1/grip_vision/confidence
ros2 topic echo /RM2/grip_vision/confidence
ros2 topic echo /RM3/grip_vision/confidence
```

确认摄像头话题：

```bash
ros2 topic list | grep camera
```

查看调试图像：

```bash
rqt_image_view
```

选择：

```bash
/RM1/grip_vision/debug_image
/RM2/grip_vision/debug_image
/RM3/grip_vision/debug_image
```

## 调参命令

```bash
ros2 run rm_grip_vision hsv_tuner --ros-args -p robot_name:=RM1
ros2 run rm_grip_vision hsv_tuner --ros-args -p robot_name:=RM2
ros2 run rm_grip_vision hsv_tuner --ros-args -p robot_name:=RM3
```

调参窗口按键：

```text
p：打印当前参数
q：退出
```

## 常用参数位置

默认参数文件：

```bash
rm_grip_vision/config/grip_vision_default.yaml
```

重点调参项：

```yaml
roi: [0.24, 0.42, 0.76, 0.96]

gray_s_max: 85
gray_v_min: 35
gray_v_max: 255

use_lab_gray: true
lab_chroma_max: 24.0
lab_l_min: 35
lab_l_max: 255

use_green_suppression: true
green_h_min: 35
green_h_max: 95
green_s_min: 45
green_v_min: 35

min_area_fraction: 0.004
max_area_fraction: 0.45
min_circularity: 0.28
confidence_threshold: 0.55
history_size: 5
required_detections: 3
```

## 参考仓库和文档

RoboMaster ROS2 驱动：

```text
https://github.com/jeguzzi/robomaster_ros
```

RoboMaster SDK 文档：

```text
https://robomaster-dev.readthedocs.io/zh-cn/latest/
```

RoboMaster SDK 示例：

```text
https://github.com/dji-sdk/RoboMaster-SDK/tree/master/examples
```

RoboMaster EP 抓网球参考：

```text
https://github.com/TaylorXin/RobomasterEp_GetTennis
https://github.com/AyemonBaraka/Robomaster_Ep_Core_Fetch_Tennis
```

RoboMaster EP 小球跟踪参考：

```text
https://github.com/MaxwellJay256/ball-tracker
```

