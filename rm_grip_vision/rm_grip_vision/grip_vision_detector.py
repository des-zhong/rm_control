"""Visual grasp-success detector for the RoboMaster EP gripper camera.

The node uses only classical image processing:
1. crop the lower centre region where the ball should remain after the gripper closes;
2. segment the ball candidate region;
   - default mode is ``gray_on_green`` for a gray motion-capture-coated tennis ball
     on a green field/background;
   - legacy HSV range mode is kept for coloured balls;
3. filter contours by area and circularity;
4. publish a stable Boolean decision after several consecutive detections.

It intentionally does not depend on a neural network. Tune ROI and the gray/green
thresholds for the actual ball material, camera mounting angle and lab lighting.
"""

from collections import deque
from typing import Iterable, List, Sequence, Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


HSVRange = Tuple[np.ndarray, np.ndarray]


def _as_int_list(value: Iterable[int], *, length: int, name: str) -> List[int]:
    values = [int(v) for v in value]
    if len(values) != length:
        raise ValueError(f"{name} must contain exactly {length} integers, got {values}")
    return values


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


class GripVisionDetector(Node):
    """Detect whether the gray reflective ball is visible in the gripper capture ROI."""

    def __init__(self) -> None:
        super().__init__('grip_vision_detector')

        self.robot_name = str(self.declare_parameter('robot_name', 'RM1').value)
        self.image_topic = str(
            self.declare_parameter(
                'image_topic', f'/{self.robot_name}/camera/image_color'
            ).value
        )

        self.caught_topic = str(
            self.declare_parameter(
                'caught_topic', f'/{self.robot_name}/grip_vision/caught'
            ).value
        )
        self.confidence_topic = str(
            self.declare_parameter(
                'confidence_topic', f'/{self.robot_name}/grip_vision/confidence'
            ).value
        )
        self.debug_image_topic = str(
            self.declare_parameter(
                'debug_image_topic', f'/{self.robot_name}/grip_vision/debug_image'
            ).value
        )

        # ROI is normalised [x_min, y_min, x_max, y_max].  The default is the
        # lower centre of the gripper camera image, where a captured ball should
        # be visible between the two gripper fingers.
        self.roi = [
            _clamp01(v)
            for v in self.declare_parameter(
                'roi', [0.24, 0.42, 0.76, 0.96]
            ).value
        ]
        if len(self.roi) != 4 or self.roi[0] >= self.roi[2] or self.roi[1] >= self.roi[3]:
            raise ValueError('roi must be [x_min, y_min, x_max, y_max] with min < max')

        # Segmentation mode:
        # - gray_on_green: default for a tennis ball coated with gray motion-capture
        #   reflective material on a green field/background.  It searches for low
        #   saturation / near-neutral regions and explicitly suppresses green pixels.
        # - hsv_ranges: legacy mode for coloured balls, using hsv_lower/hsv_upper.
        self.segmentation_mode = str(
            self.declare_parameter('segmentation_mode', 'gray_on_green').value
        )

        # Legacy HSV ranges.  Kept so older orange/red or other coloured-ball setups
        # can still be used by setting segmentation_mode:=hsv_ranges.
        self.hsv_lower = _as_int_list(
            self.declare_parameter('hsv_lower', [0, 70, 60]).value,
            length=3,
            name='hsv_lower',
        )
        self.hsv_upper = _as_int_list(
            self.declare_parameter('hsv_upper', [28, 255, 255]).value,
            length=3,
            name='hsv_upper',
        )
        self.hsv_lower2 = _as_int_list(
            self.declare_parameter('hsv_lower2', [170, 70, 60]).value,
            length=3,
            name='hsv_lower2',
        )
        self.hsv_upper2 = _as_int_list(
            self.declare_parameter('hsv_upper2', [179, 255, 255]).value,
            length=3,
            name='hsv_upper2',
        )
        self.use_second_hsv_range = bool(
            self.declare_parameter('use_second_hsv_range', False).value
        )

        # Default gray-on-green thresholds.  For gray objects, hue is unreliable;
        # saturation/chroma and brightness are more useful.  The HSV gray mask keeps
        # low-saturation pixels, while the Lab mask keeps pixels close to neutral
        # gray.  Green suppression removes the field/background.
        self.gray_s_max = int(self.declare_parameter('gray_s_max', 85).value)
        self.gray_v_min = int(self.declare_parameter('gray_v_min', 35).value)
        self.gray_v_max = int(self.declare_parameter('gray_v_max', 255).value)
        self.lab_chroma_max = float(self.declare_parameter('lab_chroma_max', 24.0).value)
        self.lab_l_min = int(self.declare_parameter('lab_l_min', 35).value)
        self.lab_l_max = int(self.declare_parameter('lab_l_max', 255).value)
        self.green_h_min = int(self.declare_parameter('green_h_min', 35).value)
        self.green_h_max = int(self.declare_parameter('green_h_max', 95).value)
        self.green_s_min = int(self.declare_parameter('green_s_min', 45).value)
        self.green_v_min = int(self.declare_parameter('green_v_min', 35).value)
        self.use_lab_gray = bool(self.declare_parameter('use_lab_gray', True).value)
        self.use_green_suppression = bool(
            self.declare_parameter('use_green_suppression', True).value
        )

        self.min_area_fraction = float(
            self.declare_parameter('min_area_fraction', 0.004).value
        )
        self.max_area_fraction = float(
            self.declare_parameter('max_area_fraction', 0.45).value
        )
        self.min_circularity = float(self.declare_parameter('min_circularity', 0.28).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.55).value
        )
        self.center_x_tolerance = float(
            self.declare_parameter('center_x_tolerance', 0.24).value
        )
        self.center_y_min = float(self.declare_parameter('center_y_min', 0.42).value)
        self.history_size = int(self.declare_parameter('history_size', 5).value)
        self.required_detections = int(
            self.declare_parameter('required_detections', 3).value
        )
        self.blur_kernel = int(self.declare_parameter('blur_kernel', 5).value)
        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1
        self.morph_kernel = int(self.declare_parameter('morph_kernel', 5).value)
        if self.morph_kernel < 1:
            self.morph_kernel = 1

        self.publish_debug = bool(self.declare_parameter('publish_debug', False).value)
        self.log_period_sec = float(self.declare_parameter('log_period_sec', 2.0).value)

        self.bridge = CvBridge()
        self.history: deque[bool] = deque(maxlen=max(1, self.history_size))
        self.last_log_time = self.get_clock().now()
        self.last_caught = False
        self.last_confidence = 0.0

        self.caught_pub = self.create_publisher(Bool, self.caught_topic, 5)
        self.confidence_pub = self.create_publisher(Float32, self.confidence_topic, 5)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, 5)

        self.get_logger().info(
            f'Grip vision detector started for {self.robot_name}: '
            f'image={self.image_topic}, caught={self.caught_topic}, roi={self.roi}'
        )

    def hsv_ranges(self) -> Sequence[HSVRange]:
        ranges: List[HSVRange] = [
            (np.array(self.hsv_lower, dtype=np.uint8), np.array(self.hsv_upper, dtype=np.uint8))
        ]
        if self.use_second_hsv_range:
            ranges.append(
                (
                    np.array(self.hsv_lower2, dtype=np.uint8),
                    np.array(self.hsv_upper2, dtype=np.uint8),
                )
            )
        return ranges

    def segment_ball_candidate(self, roi_proc: np.ndarray) -> np.ndarray:
        """Return a binary mask for the ball candidate in the gripper ROI."""
        hsv = cv2.cvtColor(roi_proc, cv2.COLOR_BGR2HSV)

        if self.segmentation_mode == 'hsv_ranges':
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in self.hsv_ranges():
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            return mask

        if self.segmentation_mode != 'gray_on_green':
            self.get_logger().warn(
                f'unknown segmentation_mode={self.segmentation_mode!r}; using gray_on_green'
            )
            self.segmentation_mode = 'gray_on_green'

        h, s, v = cv2.split(hsv)

        # Gray motion-capture coating: low saturation, not too dark.  Hue is ignored
        # because hue is unstable when saturation is low.
        hsv_gray = cv2.inRange(
            hsv,
            np.array([0, 0, self.gray_v_min], dtype=np.uint8),
            np.array([179, self.gray_s_max, self.gray_v_max], dtype=np.uint8),
        )

        gray_mask = hsv_gray
        if self.use_lab_gray:
            lab = cv2.cvtColor(roi_proc, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            chroma = np.sqrt(
                (a.astype(np.float32) - 128.0) ** 2 +
                (b.astype(np.float32) - 128.0) ** 2
            )
            lab_gray = (
                (chroma <= self.lab_chroma_max) &
                (l >= self.lab_l_min) &
                (l <= self.lab_l_max)
            ).astype(np.uint8) * 255
            # OR keeps both matte gray and reflective gray pixels that may drift in
            # either colour space under changing illumination.
            gray_mask = cv2.bitwise_or(hsv_gray, lab_gray)

        if self.use_green_suppression:
            green_mask = cv2.inRange(
                hsv,
                np.array([self.green_h_min, self.green_s_min, self.green_v_min], dtype=np.uint8),
                np.array([self.green_h_max, 255, 255], dtype=np.uint8),
            )
            gray_mask = cv2.bitwise_and(gray_mask, cv2.bitwise_not(green_mask))

        return gray_mask

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        caught_raw, confidence, debug = self.process_frame(frame)
        self.history.append(caught_raw)
        stable_count = sum(1 for item in self.history if item)
        caught = stable_count >= min(self.required_detections, len(self.history))

        self.last_caught = bool(caught)
        self.last_confidence = float(confidence)
        self.caught_pub.publish(Bool(data=self.last_caught))
        self.confidence_pub.publish(Float32(data=self.last_confidence))

        if self.publish_debug:
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            except CvBridgeError as exc:
                self.get_logger().warn(f'cv_bridge debug publish failed: {exc}')

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_period_sec:
            self.last_log_time = now
            self.get_logger().info(
                f'caught={self.last_caught}, conf={self.last_confidence:.2f}, '
                f'history={stable_count}/{len(self.history)}'
            )

    def process_frame(self, frame: np.ndarray) -> Tuple[bool, float, np.ndarray]:
        height, width = frame.shape[:2]
        x0 = int(self.roi[0] * width)
        y0 = int(self.roi[1] * height)
        x1 = int(self.roi[2] * width)
        y1 = int(self.roi[3] * height)
        roi_img = frame[y0:y1, x0:x1]
        roi_area = max(1, roi_img.shape[0] * roi_img.shape[1])

        if self.blur_kernel > 1:
            roi_proc = cv2.GaussianBlur(roi_img, (self.blur_kernel, self.blur_kernel), 0)
        else:
            roi_proc = roi_img

        mask = self.segment_ball_candidate(roi_proc)

        kernel = np.ones((self.morph_kernel, self.morph_kernel), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_score = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            area_fraction = area / roi_area
            if area_fraction < self.min_area_fraction or area_fraction > self.max_area_fraction:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 1.0:
                continue
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue
            moments = cv2.moments(contour)
            if abs(moments['m00']) < 1e-6:
                continue
            cx_roi = float(moments['m10'] / moments['m00'])
            cy_roi = float(moments['m01'] / moments['m00'])
            cx = (x0 + cx_roi) / width
            cy = (y0 + cy_roi) / height
            center_score = 1.0 - min(1.0, abs(cx - 0.5) / max(1e-6, self.center_x_tolerance))
            y_score = 1.0 if cy >= self.center_y_min else max(0.0, cy / max(1e-6, self.center_y_min))
            area_score = min(1.0, area_fraction / max(1e-6, self.min_area_fraction * 5.0))
            circularity_score = min(1.0, circularity)
            score = 0.45 * area_score + 0.30 * circularity_score + 0.15 * center_score + 0.10 * y_score
            if score > best_score:
                best_score = score
                best = (contour, area_fraction, circularity, cx, cy, score)

        caught = best_score >= self.confidence_threshold
        debug = frame.copy()
        cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 255), 2)
        if best is not None:
            contour, area_fraction, circularity, cx, cy, score = best
            contour_shifted = contour + np.array([[[x0, y0]]], dtype=contour.dtype)
            cv2.drawContours(debug, [contour_shifted], -1, (0, 255, 255), 2)
            cv2.circle(debug, (int(cx * width), int(cy * height)), 4, (0, 255, 255), -1)
            label = f'conf={score:.2f} area={area_fraction:.3f} circ={circularity:.2f}'
        else:
            label = 'conf=0.00 no valid contour'
        status = 'CAUGHT' if caught else 'NOT_CAUGHT'
        cv2.putText(debug, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(debug, label, (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        return bool(caught), float(best_score), debug


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripVisionDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
