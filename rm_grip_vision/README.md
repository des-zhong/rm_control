# rm_grip_vision 实机调试说明

## 功能

`rm_grip_vision` 用 RoboMaster EP 夹爪摄像头判断球是否被夹住。节点订阅 `/RMx/camera/image_color`，裁剪 ROI 后做颜色/灰度分割，并发布：

- `/RMx/grip_vision/caught`：最终抓取结果，`strategy.py` 用它决定是否进入 `target_code=34`
- `/RMx/grip_vision/confidence`：检测质量指标，只用于调试
- `/RMx/grip_vision/caught_score`：抓取判断分数，真正影响 caught
- `/RMx/grip_vision/debug_image`：调试图，可显示 ROI、mask、轮廓和失败原因

## 编译

```bash
colcon build --symlink-install --packages-select rm_grip_vision rmcontrol
source install/setup.bash
```

## 启动

只启动 RM1/RM2/RM3 视觉检测：

```bash
ros2 launch rm_grip_vision rm123_grip_vision.launch.py publish_debug:=true
```

视觉检测和 strategy 一起启动：

```bash
ros2 launch rm_grip_vision rm123_grip_vision_and_strategy.launch.py publish_debug:=true
```

指定配置文件：

```bash
ros2 launch rm_grip_vision rm123_grip_vision.launch.py \
  config_file:=install/rm_grip_vision/share/rm_grip_vision/config/grip_vision_default.yaml \
  publish_debug:=true
```

## 调参工具

推荐使用新命令：

```bash
ros2 run rm_grip_vision segmentation_tuner --ros-args \
  -p robot_name:=RM1 \
  --params-file install/rm_grip_vision/share/rm_grip_vision/config/grip_vision_default.yaml
```

旧命令仍兼容，但不推荐新用户继续使用：

```bash
ros2 run rm_grip_vision hsv_tuner --ros-args -p robot_name:=RM1
```

`segmentation_tuner` 里先调 ROI，再调分割模式和颜色阈值。按 `p` 打印当前参数，复制到 `rm_grip_vision/config/grip_vision_default.yaml`。

## 查看 Topic

```bash
ros2 topic echo /RM1/grip_vision/caught
ros2 topic echo /RM1/grip_vision/confidence
ros2 topic echo /RM1/grip_vision/caught_score
ros2 run rqt_image_view rqt_image_view /RM1/grip_vision/debug_image
```

三类输出区别：

- `confidence`：检测质量分数，只用于看 mask 和轮廓是否像目标，不决定最终抓取
- `caught_score`：抓取判断分数，由面积、水平位置、纵向位置计算
- `caught`：最终布尔结果，由单帧 caught 判断和多帧稳定共同决定

## 关键参数

`segmentation_mode`：

- `gray_on_green`：默认模式，适合灰色/银灰色反光网球 + 绿色场地
- `bgr_ranges`：OpenCV 原图 BGR 阈值，顺序是 `[B, G, R]`
- `rgb_ranges`：先转 RGB，再按 `[R, G, B]` 阈值
- `hsv_ranges`：旧彩色目标模式，不是当前灰色球首选

`roi: [x_min, y_min, x_max, y_max]`：

- 归一化比例，范围 0.0 到 1.0
- 尽量只覆盖夹爪闭合后球可能出现的位置
- 如果夹爪和球颜色接近，优先缩小 ROI

灰色球参数：

- `gray_s_max`：灰色候选最大饱和度
- `gray_v_min / gray_v_max`：灰色候选亮度范围
- `lab_chroma_max`：Lab 中允许偏离中性灰的程度
- `lab_l_min / lab_l_max`：Lab 亮度范围

绿色背景抑制：

- `green_h_min / green_h_max`
- `green_s_min`
- `green_v_min`

BGR/RGB：

- `bgr_lower / bgr_upper`：OpenCV BGR 阈值，切换到 `bgr_ranges` 前必须先调，否则默认会选中整块 ROI
- `rgb_lower / rgb_upper`：RGB 阈值，仅 `rgb_ranges` 使用

检测质量：

- `min_area_fraction / max_area_fraction`：影响 `confidence` 的面积范围
- `center_x_tolerance / center_y_min`：影响 `confidence` 的位置评分

抓取判断：

- `caught_min_area_fraction / caught_max_area_fraction`：真正影响 caught 的面积范围
- `caught_center_x_tolerance`：球中心允许偏离 ROI 中线的程度
- `caught_center_y_min`：球中心最低纵向位置
- `caught_allow_low_circularity / caught_min_circularity`：允许被夹爪遮挡后的低圆度轮廓
- `min_circularity`：当 `caught_allow_low_circularity=false` 时使用的严格圆度阈值
- `caught_score_threshold`：单帧抓取分数阈值
- `caught_required_frames`：需要连续或窗口内多少帧满足
- `history_size`：稳定判断窗口长度

## 推荐调参流程

1. 确认摄像头 topic 正常：`ros2 topic list | grep camera`
2. 启动 `segmentation_tuner`
3. 先调 ROI，让白框只覆盖夹爪中球可能出现的位置
4. 默认先试 `gray_on_green`
5. 如果原图中 BGR/RGB 差异更明显，再切到 `bgr_ranges` 或 `rgb_ranges`
6. 看 `final_mask`：有球时应稳定选中球，没有球时不应选中夹爪或绿色背景
7. 看 `/caught_score` 是否随球进入夹爪明显升高
8. 看 `/caught` 是否稳定
9. 最后接入 strategy 做抓取闭环测试

## ROI 调参提示

ROI 越大，越容易把夹爪、地面、背景带进来；ROI 越小，越容易漏掉被夹住的球。调好 ROI 后按 `p`，把打印出来的 `roi: [...]` 复制回 config。

## 回退

关闭视觉抓取判断，回到动捕 `check_catched()`：

```bash
ros2 run rmcontrol strategy --ros-args -p use_visual_catch:=false
```

关闭 LOS 接近，回到原接近逻辑：

```bash
ros2 launch rm_grip_vision rm123_grip_vision_and_strategy.launch.py \
  use_los_catch_approach:=false
```
