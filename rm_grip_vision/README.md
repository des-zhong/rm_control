# rm_grip_vision

This package replaces the motion-capture based *grasp success* judgement in `rmcontrol.strategy` with a visual detector for RoboMaster EP gripper cameras.

The current default detector is designed for the updated scene:

- green field/background;
- tennis ball coated with gray motion-capture reflective material;
- no neural-network inference by default.

The detector subscribes to `/RMx/camera/image_color`, crops the lower-centre gripper region, segments a gray ball candidate while suppressing green background pixels, filters contours by area/circularity, and publishes:

- `/RMx/grip_vision/caught` (`std_msgs/Bool`)
- `/RMx/grip_vision/confidence` (`std_msgs/Float32`)
- `/RMx/grip_vision/debug_image` (`sensor_msgs/Image`, optional)

The patched `rmcontrol/rmcontrol/strategy.py` subscribes to `/RM1..3/grip_vision/caught`; after the gripper is closed, it waits briefly for a fresh visual result. If the detector confirms the ball, strategy switches to code `34`; otherwise it returns to code `30` and tries again. The old motion-capture `check_catched()` path remains available with `use_visual_catch:=false`.

## Segmentation logic for gray ball on green background

Because the tennis ball is coated by gray motion-capture reflective material, hue is no longer a reliable feature. The detector therefore uses `segmentation_mode:=gray_on_green`:

```text
ROI crop
  -> HSV low-saturation gray candidate
  -> optional Lab near-neutral gray candidate
  -> green background suppression
  -> morphology open/close
  -> contour area + circularity + ROI-position score
  -> multi-frame stable caught decision
```

Important parameters:

```yaml
segmentation_mode: gray_on_green

# Gray candidate: low saturation / near-neutral colour.
gray_s_max: 85
gray_v_min: 35
gray_v_max: 255
use_lab_gray: true
lab_chroma_max: 24.0
lab_l_min: 35
lab_l_max: 255

# Green field/background suppression.
use_green_suppression: true
green_h_min: 35
green_h_max: 95
green_s_min: 45
green_v_min: 35
```

If the detector misses the gray ball, usually increase `gray_s_max` or `lab_chroma_max`, or lower `gray_v_min`. If it detects the green field/background, increase `green_s_min` slightly or narrow `green_h_min/green_h_max`. If it detects black gripper parts, tighten `roi`, raise `gray_v_min`, or raise `min_circularity`.

## Build

Place this repository in your ROS 2 workspace `src` together with `robomaster_ros`, then build:

```bash
cd <ros2_ws>
source /opt/ros/<ROS_DISTRO>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Install runtime dependencies if missing:

```bash
sudo apt install ros-<ROS_DISTRO>-cv-bridge python3-opencv python3-numpy
```

## Run

Start the robots as before:

```bash
ros2 launch rmcontrol ep_startup.launch.py
ros2 launch rmcontrol ep_control.launch.py
```

Then launch the visual detectors and the visual-enabled strategy:

```bash
ros2 launch rm_grip_vision rm123_grip_vision_and_strategy.launch.py publish_debug:=true
```

Alternatively, if you prefer to keep your current `ros2 run rmcontrol strategy` terminal, start only the detectors:

```bash
ros2 launch rm_grip_vision rm123_grip_vision.launch.py publish_debug:=true
ros2 run rmcontrol strategy --ros-args -p use_visual_catch:=true
```

## Tune gray/green segmentation

Run the tuner on a machine with a display:

```bash
ros2 run rm_grip_vision hsv_tuner --ros-args -p robot_name:=RM1
```

The tuner shows:

- `gray_candidate`: low-saturation / near-neutral candidate pixels;
- `green_background_suppressed`: pixels treated as green background;
- `final_mask`: candidate mask after green suppression and morphology;
- `camera_with_roi`: original camera image with ROI overlay.

Keys:

- `p`: print parameters you can copy into launch/config.
- `q`: quit.

Useful detector launch example:

```bash
ros2 run rm_grip_vision grip_vision_detector --ros-args \
  -p robot_name:=RM1 \
  -p image_topic:=/RM1/camera/image_color \
  -p segmentation_mode:=gray_on_green \
  -p gray_s_max:=85 \
  -p gray_v_min:=35 \
  -p lab_chroma_max:=24 \
  -p green_h_min:=35 \
  -p green_h_max:=95 \
  -p roi:="[0.24, 0.42, 0.76, 0.96]" \
  -p publish_debug:=true
```

## Legacy coloured-ball mode

If you switch back to a coloured ball, set:

```bash
-p segmentation_mode:=hsv_ranges \
-p hsv_lower:="[0, 70, 60]" \
-p hsv_upper:="[28, 255, 255]"
```

## Optional neural-network fallback

The gray-on-green detector is the intended solution. If lighting/background makes classical segmentation unreliable, `scripts/train_yolo_ball.py` is included as a fallback training helper:

```bash
python3 -m pip install ultralytics
python3 src/rm_control-main/rm_grip_vision/scripts/train_yolo_ball.py \
  --data src/rm_control-main/soccervision.v3i.yolov8/data.yaml \
  --model yolov8n.pt --epochs 80 --imgsz 640 --class-id 0
```

Before using this fallback, confirm that your dataset actually contains labelled ball boxes for class `0` (`ball1`) in the label files.
