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
ros2 run roscpp pub_pos # obastacle avoidance
ros2 launch rmcontrol ep_startup.launch.py 
ros2 launch rmcontrol s1_startup.launch.py 
ros2 launch yscontrol ys_control.launch.py 
ros2 run rmcontrol strategy 
ros2 run rmcontrol data_recorder
ros2 run rmcontrol camera_recorder # 两台机器狗相机，每秒保存一次彩色图和真实深度图
python3 as2_follow/run_both.py --enable-motion --confirm-field-x --accept-unstamped-mocap
ros2 launch rmcontrol ep_control.launch.py
```
To stop RMs, stop strategy node and ep_startup.launch.py and s1_startup.launch.py.

以上长驻命令分别在独立终端运行。相机记录器放在 `data_recorder` 之后、机器狗及 RM
运动控制之前启动；从本仓库 `src` 目录运行，让数据保存在当前目录的 `run_data`。
新增入口首次使用需在工作区根目录运行 `colcon build --packages-select rmcontrol`，
并在运行终端 `source install/setup.bash`。本机依赖 `python3-paramiko`、`python3-opencv`、
`python3-numpy`（可通过 `sudo apt install python3-paramiko python3-opencv python3-numpy` 安装）。

相机记录器默认连接 `unitree@172.20.10.13` 和 `unitree@172.20.10.14`，密码均为 `123`，
无需输入。两台狗独立记录，未连接的狗每 10 秒重试，上线后自动加入，不影响另一台。
默认每秒请求、保存一组图像；RealSense 底层视频流保持 30 FPS，网络或写盘跟不上时
会记录 `slow_capture` 并降低实际保存频率，不补采旧帧。按 Ctrl+C 停止。
狗端需要已有 `g++`、`pkg-config`、`librealsense2` 和 OpenCV4 开发库；13 已实测。

```bash
ros2 run rmcontrol camera_recorder
# 只录当前在线的13；可选调节采集间隔或保存路径
ros2 run rmcontrol camera_recorder --hosts 172.20.10.13 --interval 1 --output-dir run_data
```

每次启动创建 `run_data/YYYYMMDD_HHMMSS_微秒/`，与 `data_recorder` 使用相同的
会话命名方式，但为独立会话目录，不自动合并。目录内结构为：

```text
camera_recording.json
camera_events.jsonl
cameras/<机器狗IP>/camera_<相机序列号>/<采集日期时间>/
  color.jpg
  depth.png
  depth_preview.png
  metadata.json
```

彩色图以 JPEG 质量 95 压缩传输并原样保存，深度图使用无损 PNG。
`depth.png` 为对齐到彩色图的 16 位真实深度：像素值乘 `metadata.json` 中的
`depth_scale_m` 得到米，0 为无效深度。预览图固定显示 0～5 米，无效点为黑色。
元数据记录相机型号、序列号、内参、设备时间戳、主机请求/接收 Unix 纳秒时间和有效深度点数；
可用主机 Unix 时间与 `data_recorder` 数据关联。设备时间戳不一定为 Unix 时间，
两台狗之间不保证同步。图像通过 SSH 返回本机保存，不写入 ROS bag。
设备没有可用 RealSense 或流被占用时记录错误并重试，不从彩色图估算深度。



### AS2 双机器狗：恢复动捕后一条命令启动

在已加载 ROS 和本工作区 `install/setup.bash` 环境的独立终端中，从本仓库 `src` 目录执行。
**执行时机：** 先启动动捕并确认 `/motions` 正常、球位置有效；将六台 RM 按初始分组摆好
（X 正负半场各三台）。按原流程启动其他节点和数据记录器，看到 `Recording ready` 后，
在启动 `ros2 launch rmcontrol ep_control.launch.py`、让队员正式运动之前，执行下面的双狗命令。
不运行机器狗模块时，原比赛流程照常执行。

启动前，下方狗 `172.20.10.13` 放在 `(x=0, y=-2.5 m)`，朝 +Y；
上方狗 `172.20.10.14` 放在 `(x=0, y=+2.5 m)`，朝 -Y，均已正常站立。
确认场地长轴为动捕 X，并知晓原 `/motions` 无源时间戳/有效性标记的限制后，一条命令启用两台狗：

```bash
python3 as2_follow/run_both.py --enable-motion --confirm-field-x --accept-unstamped-mocap
```

两台狗自动使用固定 SSH 密码 `123`，无需输入；IP 可在 `as2_follow/run_both.py` 顶部修改，
或通过 `--lower-ip`、`--upper-ip` 覆盖。分别看到 `as2_follow_lower` 和 `as2_follow_upper`
打印 `Locked team`，确认分组正确后，再启动 RM 控制进入比赛。
下方狗固定跟随初始化在 X 负半场的三台 RM，上方狗固定跟随正半场的三台 RM；
沿 X 跟随队员均值、保持各自 Y 并朝向球，默认平移限速 0.12 m/s。

首次恢复动捕时，可先用同一入口预览（不启用运动）：

```bash
python3 as2_follow/run_both.py
```

结束预览后将狗保持/恢复到上述初始位置，再运行实际跟随命令。
**在统一启动终端按 Ctrl+C，会同时停止两个跟随进程**，各自狗端处理连接结束和指令超时并请求停止。
两台狗仍使用独立进程和 SSH 连接，一台故障不会关闭另一台，也不会停止原比赛节点；
现场需确认停稳。日志分别在 `run_data/as2_follow/lower/` 和 `run_data/as2_follow/upper/`。
更多参数和定位限制见 [AS2 模块说明](as2_follow/README.md)。

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
ros2 run rmcontrol data_recorder # 新终端运行，看到 Recording ready 后继续
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
ros2 run rmcontrol data_recorder # 新终端运行，看到 Recording ready 后继续
ros2 launch rmcontrol ep_control.launch.py
```

按以上顺序启动时，策略已给抓球队员下发 `code=30`。该机器人直接执行
`Lower arm`（绝对目标 `x=0.17, z=-0.08`）→ `Open gripper` → 接近球，
不先执行 `Startup raise arm`。其他 RM1–RM3 队员仍执行抬臂初始化。
看到 `approach enabled, status=30` 表示放下、开爪已经完成；
若失败，请保留对应机器人的 `Lower arm` 请求、进度和结果日志。

## 分开启动视觉检测和策略节点

```bash
ros2 launch rm_grip_vision rm123_grip_vision.launch.py publish_debug:=true
ros2 run rmcontrol strategy --ros-args -p use_visual_catch:=true
ros2 run rmcontrol data_recorder # 新终端运行，看到 Recording ready 后继续
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

查看检测置信度：ros2 run roscpp pub_pos

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
ros2 run rm_grip_vision hsv_tuner --ros-argros2 run rm_grip_vision hsv_tuner --ros-args -p robot_name:=RM2s -p robot_name:=RM2
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
ros2 launch rm_grip_vision rm123_grip_vision_and_strategy.launch.py publish_debug:=true
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



## 独立实验数据记录

记录节点仅订阅现有话题，不发送控制指令、不调用机器人服务，不改变控制逻辑。
每次运行在当前目录的 `run_data/YYYYMMDD_HHMMSS_ffffff/` 创建独立目录，
时间采用运行电脑的本地时区，记录电脑应与手持相机校准时间。
请在策略/视觉节点启动后、`ep_control.launch.py` 启动前，在独立终端运行：

```bash
ros2 run rmcontrol data_recorder
```

看到 `Recording ready: ...` 后再启动 RM 控制。节点每秒发现新话题，
支持控制节点稍后启动。实验结束后先停止机器人控制，再在记录终端按 Ctrl+C，
让 bag 完成写入。每次启动记录节点对应一次记录会话。

可指定保存根目录（建议使用有足够空间的本地磁盘）：

```bash
ros2 run rmcontrol data_recorder --ros-args -p output_dir:=/absolute/path/run_data
```

新增依赖及编译：

```bash
sudo apt install ros-galactic-rosbag2-py ros-galactic-rosbag2-storage-default-plugins ros-galactic-rosidl-runtime-py
colcon build --symlink-install --packages-select rmcontrol
source install/setup.bash
```

每次会话包含：

- `bag/`：原始 ROS 消息，包含全程 `/motions`、`/to_rvo`、`/rvo2`、`/ctrl`、`/yscomm`、
  `/rosout`，以及 RM1–6 的底盘指令、执行状态、机械臂位置、夹爪状态和已发布的机械臂/夹爪 action 反馈与状态。
  同时保存已发布的摄像头原始图像，以及抓取视觉 caught、confidence、caught_score 和调试图像；
  不抽帧、不压缩，原始图像与调试图像可能占用较大空间。
- `trajectory.csv`：球和机器人的逐消息平面轨迹与原样速度，缺失速度留空。
- `messages.jsonl`：除图像外的消息全文及接收时间，便于分析指令和抓取证据。
- `events.csv`：命令、执行状态、视觉抓取状态的变化，以及订阅和记录起止事件。
- `metadata.json`：开始/结束时间、实际订阅话题及类型、收到的消息数、数据解释说明。

CSV/JSONL 同时记录 Unix、单调时钟及 ROS 接收时间（纳秒）。
图像的源时间戳保留在 bag 原始消息中；动捕消息当前没有源时间戳。
使用 `ros2 bag info <会话目录>/bag` 检查记录内容；原始图像可从 bag 离线提取。

事件表示观察到的请求或状态，不将闭爪、视觉判定、踢球通知直接认作物理成功。
例如 `catch_accepted_transport_requested` 是策略接受抓取并请求运输，
`ys_kick_requested` 是踢球通知，真实出球、进球和摔倒需结合轨迹及人工场地视频确认。
记录器不会额外录制场地视频。

轨迹实体名称按现有约定解释：索引 0 为球，1–6 为 RM，随后为 Yanshee；
请核对动捕刚体顺序，原始索引始终保留。动捕平移速度的现有缩放问题不在记录器中修正，
分析时建议从位置与时间独立估算。

记录器采用 best-effort、volatile 订阅以兼容现有发布端，
不补录启动前的数据；高图像流量或磁盘速度不足可能丢消息。
每秒刷新 CSV/JSONL，异常断电仍可能导致尾部数据或 bag 元数据缺失。


### 将运行数据绘制成图片

```bash
python3 -m pip install matplotlib
python3 run_data/visualize.py <时间戳文件夹名或完整路径>
```

图片直接保存在对应会话文件夹内，详见 [可视化说明](run_data/README.md)。
