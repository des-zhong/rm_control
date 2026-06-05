"""RoboMaster EP 夹爪摄像头视觉抓取判断节点。

The node uses classical image processing:
1. crop the gripper ROI where the ball should remain after the gripper closes;
2. segment the ball candidate region;
   - ``gray_on_green`` keeps the original HSV/Lab gray detector with green suppression;
   - ``bgr_ranges`` thresholds the OpenCV BGR image directly;
   - ``rgb_ranges`` thresholds an RGB-converted copy for users who prefer RGB values;
   - ``hsv_ranges`` keeps the legacy coloured-ball HSV mode;
3. compute ``detection_confidence`` for debugging and the confidence topic;
4. compute a separate ``caught_score`` and ``caught_decision`` that favour stable
   target presence in the gripper ROI over perfect circularity.

OpenCV camera frames are BGR by default. Parameters named ``bgr_*`` are applied
directly to the image from cv_bridge; parameters named ``rgb_*`` are applied after
converting that ROI to RGB.
"""

from collections import deque
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

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
        raise ValueError(f'{name} must contain exactly {length} integers, got {values}')
    return [max(0, min(255, v)) for v in values]


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


class GripVisionDetector(Node):
    """Detect whether the ball is visibly present in the gripper capture ROI."""

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
        self.caught_score_topic = str(
            self.declare_parameter(
                'caught_score_topic', f'/{self.robot_name}/grip_vision/caught_score'
            ).value
        )
        self.debug_image_topic = str(
            self.declare_parameter(
                'debug_image_topic', f'/{self.robot_name}/grip_vision/debug_image'
            ).value
        )

        self.roi = [
            _clamp01(v)
            for v in self.declare_parameter(
                'roi', [0.24, 0.42, 0.76, 0.96]
            ).value
        ]
        if len(self.roi) != 4 or self.roi[0] >= self.roi[2] or self.roi[1] >= self.roi[3]:
            raise ValueError('roi must be [x_min, y_min, x_max, y_max] with min < max')

        self.segmentation_mode = str(
            self.declare_parameter('segmentation_mode', 'gray_on_green').value
        ).lower()

        # hsv_ranges：旧彩色球方案，当前灰色/银灰反光网球通常不用它。
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

        # bgr_ranges/rgb_ranges：实机原图里球和夹爪差异明显时更直观。
        # 注意 cv_bridge/OpenCV 默认是 BGR；rgb_ranges 会先转换成 RGB 再阈值。
        self.bgr_lower = _as_int_list(
            self.declare_parameter('bgr_lower', [0, 0, 0]).value,
            length=3,
            name='bgr_lower',
        )
        self.bgr_upper = _as_int_list(
            self.declare_parameter('bgr_upper', [255, 255, 255]).value,
            length=3,
            name='bgr_upper',
        )
        self.rgb_lower = _as_int_list(
            self.declare_parameter('rgb_lower', [0, 0, 0]).value,
            length=3,
            name='rgb_lower',
        )
        self.rgb_upper = _as_int_list(
            self.declare_parameter('rgb_upper', [255, 255, 255]).value,
            length=3,
            name='rgb_upper',
        )

        # gray_on_green：默认模式。灰/银灰球用低饱和度和 Lab 近中性判断，
        # 再用绿色抑制排除场地背景。
        self.gray_s_max = int(self.declare_parameter('gray_s_max', 85).value)
        self.gray_v_min = int(self.declare_parameter('gray_v_min', 35).value)
        self.gray_v_max = int(self.declare_parameter('gray_v_max', 255).value)
        self.use_lab_gray = bool(self.declare_parameter('use_lab_gray', True).value)
        self.lab_chroma_max = float(self.declare_parameter('lab_chroma_max', 24.0).value)
        self.lab_l_min = int(self.declare_parameter('lab_l_min', 35).value)
        self.lab_l_max = int(self.declare_parameter('lab_l_max', 255).value)

        self.use_green_suppression = bool(
            self.declare_parameter('use_green_suppression', True).value
        )
        self.green_h_min = int(self.declare_parameter('green_h_min', 35).value)
        self.green_h_max = int(self.declare_parameter('green_h_max', 95).value)
        self.green_s_min = int(self.declare_parameter('green_s_min', 45).value)
        self.green_v_min = int(self.declare_parameter('green_v_min', 35).value)

        # detection_confidence 只用于调试和 /confidence topic，不决定最终 caught。
        self.min_area_fraction = float(
            self.declare_parameter('min_area_fraction', 0.004).value
        )
        self.max_area_fraction = float(
            self.declare_parameter('max_area_fraction', 0.45).value
        )
        self.min_circularity = float(self.declare_parameter('min_circularity', 0.25).value)
        self.center_x_tolerance = float(
            self.declare_parameter('center_x_tolerance', 0.24).value
        )
        self.center_y_min = float(self.declare_parameter('center_y_min', 0.42).value)

        # caught_score / caught_decision：最终抓取判断用这组参数。它更关心
        # “夹爪 ROI 中是否稳定存在目标”，而不是轮廓是否完美圆形。
        self.caught_min_area_fraction = float(
            self.declare_parameter('caught_min_area_fraction', self.min_area_fraction).value
        )
        self.caught_max_area_fraction = float(
            self.declare_parameter('caught_max_area_fraction', self.max_area_fraction).value
        )
        self.caught_center_x_tolerance = float(
            self.declare_parameter('caught_center_x_tolerance', 0.28).value
        )
        self.caught_center_y_min = float(
            self.declare_parameter('caught_center_y_min', 0.42).value
        )
        self.caught_allow_low_circularity = bool(
            self.declare_parameter('caught_allow_low_circularity', True).value
        )
        self.caught_min_circularity = float(
            self.declare_parameter('caught_min_circularity', 0.06).value
        )
        self.caught_score_threshold = float(
            self.declare_parameter('caught_score_threshold', 0.45).value
        )

        self.history_size = int(self.declare_parameter('history_size', 5).value)
        # required_detections 是旧参数名，仅为兼容旧 launch/命令行保留；
        # 新配置请使用 caught_required_frames。
        legacy_required = int(self.declare_parameter('required_detections', 3).value)
        self.caught_required_frames = int(
            self.declare_parameter('caught_required_frames', legacy_required).value
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
        self.history = deque(maxlen=max(1, self.history_size))
        self.last_log_time = self.get_clock().now()
        self.last_caught = False
        self.last_confidence = 0.0
        self.last_caught_score = 0.0

        self.caught_pub = self.create_publisher(Bool, self.caught_topic, 5)
        self.confidence_pub = self.create_publisher(Float32, self.confidence_topic, 5)
        self.caught_score_pub = self.create_publisher(Float32, self.caught_score_topic, 5)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, 5)

        self.get_logger().info(
            f'Grip vision detector started for {self.robot_name}: '
            f'image={self.image_topic}, caught={self.caught_topic}, '
            f'caught_score={self.caught_score_topic}, '
            f'mode={self.segmentation_mode}, roi={self.roi}'
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

    def green_mask(self, hsv: np.ndarray) -> np.ndarray:
        """绿色场地抑制 mask。该 mask 会从候选目标中扣掉。"""
        return cv2.inRange(
            hsv,
            np.array([self.green_h_min, self.green_s_min, self.green_v_min], dtype=np.uint8),
            np.array([self.green_h_max, 255, 255], dtype=np.uint8),
        )

    def suppress_green(self, mask: np.ndarray, hsv: np.ndarray) -> np.ndarray:
        """可选绿色背景抑制，避免绿色场地被当成球。"""
        if not self.use_green_suppression:
            return mask
        return cv2.bitwise_and(mask, cv2.bitwise_not(self.green_mask(hsv)))

    def segment_ball_candidate(self, roi_proc: np.ndarray) -> np.ndarray:
        """根据 segmentation_mode 生成球候选 mask，后续共用形态学和轮廓判断。"""
        hsv = cv2.cvtColor(roi_proc, cv2.COLOR_BGR2HSV)
        mode = self.segmentation_mode

        if mode == 'hsv_ranges':
            # HSV 彩色目标模式：保留旧流程，主要用于非当前灰色反光球。
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in self.hsv_ranges():
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            return self.suppress_green(mask, hsv)

        if mode == 'bgr_ranges':
            # OpenCV 原图是 BGR 顺序，这里直接按 [B, G, R] 阈值分割。
            mask = cv2.inRange(
                roi_proc,
                np.array(self.bgr_lower, dtype=np.uint8),
                np.array(self.bgr_upper, dtype=np.uint8),
            )
            return self.suppress_green(mask, hsv)

        if mode == 'rgb_ranges':
            # 用户如果按 RGB 读数调参，先从 BGR 转 RGB，再按 [R, G, B] 阈值分割。
            roi_rgb = cv2.cvtColor(roi_proc, cv2.COLOR_BGR2RGB)
            mask = cv2.inRange(
                roi_rgb,
                np.array(self.rgb_lower, dtype=np.uint8),
                np.array(self.rgb_upper, dtype=np.uint8),
            )
            return self.suppress_green(mask, hsv)

        if mode != 'gray_on_green':
            self.get_logger().warn(
                f'unknown segmentation_mode={self.segmentation_mode!r}; using gray_on_green'
            )
            self.segmentation_mode = 'gray_on_green'

        # 灰/银灰反光球：HSV 中低饱和度 + Lab 中接近中性灰。
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
            gray_mask = cv2.bitwise_or(hsv_gray, lab_gray)

        return self.suppress_green(gray_mask, hsv)

    def image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        frame_decision, detection_confidence, state = self.process_frame(frame)
        self.history.append(frame_decision)
        stable_count = sum(1 for item in self.history if item)
        required = min(max(1, self.caught_required_frames), self.history.maxlen)
        caught_decision = len(self.history) >= required and stable_count >= required

        self.last_caught = bool(caught_decision)
        self.last_confidence = float(detection_confidence)
        self.last_caught_score = float(state.get('caught_score', 0.0))
        self.caught_pub.publish(Bool(data=self.last_caught))
        self.confidence_pub.publish(Float32(data=self.last_confidence))
        self.caught_score_pub.publish(Float32(data=self.last_caught_score))

        if self.publish_debug:
            debug = self.build_debug_image(frame, state, self.last_caught, stable_count, required)
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            except CvBridgeError as exc:
                self.get_logger().warn(f'cv_bridge debug publish failed: {exc}')

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_period_sec:
            self.last_log_time = now
            self.get_logger().info(
                f'视觉抓取: caught={self.last_caught}, frame_decision={frame_decision}, '
                f'confidence={self.last_confidence:.2f}, '
                f'caught_score={self.last_caught_score:.2f}, '
                f'history={stable_count}/{len(self.history)}'
            )

    def contour_metrics(
        self,
        contour: np.ndarray,
        roi_area: int,
        x0: int,
        y0: int,
        width: int,
        height: int,
    ) -> Optional[Dict[str, Any]]:
        area = float(cv2.contourArea(contour))
        area_fraction = area / max(1, roi_area)
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 1.0:
            return None
        moments = cv2.moments(contour)
        if abs(moments['m00']) < 1e-6:
            return None
        cx_roi = float(moments['m10'] / moments['m00'])
        cy_roi = float(moments['m01'] / moments['m00'])
        cx = (x0 + cx_roi) / max(1, width)
        cy = (y0 + cy_roi) / max(1, height)
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        return {
            'contour': contour,
            'area_fraction': area_fraction,
            'circularity': circularity,
            'cx': cx,
            'cy': cy,
        }

    def detection_score(self, metrics: Dict[str, Any]) -> float:
        area_fraction = float(metrics['area_fraction'])
        circularity = float(metrics['circularity'])
        cx = float(metrics['cx'])
        cy = float(metrics['cy'])
        center_score = 1.0 - min(1.0, abs(cx - 0.5) / max(1e-6, self.center_x_tolerance))
        y_score = 1.0 if cy >= self.center_y_min else max(0.0, cy / max(1e-6, self.center_y_min))
        area_score = min(1.0, area_fraction / max(1e-6, self.min_area_fraction * 5.0))
        circularity_score = min(1.0, max(0.0, circularity))
        return (
            0.45 * area_score +
            0.30 * circularity_score +
            0.15 * center_score +
            0.10 * y_score
        )

    def caught_score(self, metrics: Dict[str, Any]) -> float:
        """抓取判断分数：面积、水平居中和纵向位置共同决定。"""
        area_fraction = float(metrics['area_fraction'])
        cx = float(metrics['cx'])
        cy = float(metrics['cy'])
        area_score = min(1.0, area_fraction / max(1e-6, self.caught_min_area_fraction * 4.0))
        center_score = 1.0 - min(
            1.0,
            abs(cx - 0.5) / max(1e-6, self.caught_center_x_tolerance),
        )
        y_score = 1.0 if cy >= self.caught_center_y_min else 0.0
        return 0.50 * area_score + 0.30 * center_score + 0.20 * y_score

    def evaluate_caught_candidate(self, metrics: Dict[str, Any]) -> Dict[str, Any]:
        """计算 caught 候选的每个判定项，debug 图直接显示这些 OK/FAIL。

        这里不再使用 detection_confidence。confidence 只表示检测质量；最终
        /caught 由面积、位置、圆度下限、caught_score 和多帧稳定共同决定。
        """
        area_fraction = float(metrics['area_fraction'])
        cx = float(metrics['cx'])
        cy = float(metrics['cy'])
        circularity = float(metrics['circularity'])
        score = self.caught_score(metrics)
        x_ok = abs(cx - 0.5) <= self.caught_center_x_tolerance
        y_ok = cy >= self.caught_center_y_min
        area_ok = (
            area_fraction >= self.caught_min_area_fraction and
            area_fraction <= self.caught_max_area_fraction
        )
        if self.caught_allow_low_circularity:
            circularity_ok = circularity >= self.caught_min_circularity
        else:
            circularity_ok = circularity >= self.min_circularity
        score_ok = score >= self.caught_score_threshold
        decision = area_ok and x_ok and y_ok and circularity_ok and score_ok
        return {
            'area_ok': area_ok,
            'x_ok': x_ok,
            'y_ok': y_ok,
            'circularity_ok': circularity_ok,
            'score_ok': score_ok,
            'decision': decision,
            'caught_score': score,
        }

    def is_caught_candidate(self, metrics: Dict[str, Any]) -> bool:
        """兼容旧调用：返回单帧是否满足 caught 候选条件。"""
        return bool(self.evaluate_caught_candidate(metrics)['decision'])

    def process_frame(self, frame: np.ndarray) -> Tuple[bool, float, Dict[str, Any]]:
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

        best_detection: Optional[Dict[str, Any]] = None
        best_caught: Optional[Dict[str, Any]] = None
        best_candidate: Optional[Dict[str, Any]] = None
        best_detection_confidence = 0.0
        best_candidate_score = 0.0

        # 预过滤只做很宽松的轮廓范围限制。这里取 detection 和 caught 两套
        # 面积参数的并集，避免 caught_min_area_fraction 调小后先被
        # min_area_fraction 提前卡死。
        pre_min_area = min(self.min_area_fraction, self.caught_min_area_fraction)
        pre_max_area = max(self.max_area_fraction, self.caught_max_area_fraction)

        for contour in contours:
            metrics = self.contour_metrics(contour, roi_area, x0, y0, width, height)
            if metrics is None:
                continue
            area_fraction = float(metrics['area_fraction'])
            if area_fraction < pre_min_area or area_fraction > pre_max_area:
                continue

            confidence = self.detection_score(metrics)
            metrics['detection_confidence'] = confidence
            candidate_state = self.evaluate_caught_candidate(metrics)
            metrics['caught_score'] = candidate_state['caught_score']
            metrics['candidate_state'] = candidate_state

            detection_area_ok = (
                area_fraction >= self.min_area_fraction and
                area_fraction <= self.max_area_fraction
            )
            if detection_area_ok and confidence > best_detection_confidence:
                best_detection_confidence = confidence
                best_detection = metrics

            score = float(metrics['caught_score'])
            if best_candidate is None or score > best_candidate_score:
                best_candidate_score = score
                best_candidate = metrics

            if candidate_state['decision'] and (
                best_caught is None or score > float(best_caught['caught_score'])
            ):
                best_caught = metrics

        debug_candidate = best_caught if best_caught is not None else best_candidate
        if best_detection is None and debug_candidate is not None:
            best_detection_confidence = float(debug_candidate.get('detection_confidence', 0.0))

        state = {
            'x0': x0,
            'y0': y0,
            'x1': x1,
            'y1': y1,
            'mask': mask,
            'best_detection': best_detection,
            'best_caught': best_caught,
            'debug_candidate': debug_candidate,
            'detection_confidence': best_detection_confidence,
            'caught_score': float(debug_candidate.get('caught_score', 0.0)) if debug_candidate else 0.0,
            'frame_caught': best_caught is not None,
        }
        return bool(best_caught is not None), float(best_detection_confidence), state

    def build_debug_image(
        self,
        frame: np.ndarray,
        state: Dict[str, Any],
        caught_decision: bool,
        stable_count: int,
        required: int,
    ) -> np.ndarray:
        debug = frame.copy()
        height, width = frame.shape[:2]
        x0 = int(state['x0'])
        y0 = int(state['y0'])
        x1 = int(state['x1'])
        y1 = int(state['y1'])
        mask = state['mask']

        if mask.size > 0:
            mask_bool = mask > 0
            roi_debug = debug[y0:y1, x0:x1]
            tint = np.zeros_like(roi_debug)
            tint[:, :] = (0, 180, 255)
            roi_debug[mask_bool] = (
                roi_debug[mask_bool].astype(np.float32) * 0.45 +
                tint[mask_bool].astype(np.float32) * 0.55
            ).astype(np.uint8)
            debug[y0:y1, x0:x1] = roi_debug

        cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 255), 2)
        best_detection = state.get('best_detection')
        best_caught = state.get('best_caught')
        debug_candidate = state.get('debug_candidate')
        if best_detection is not None:
            contour = best_detection['contour']
            contour_shifted = contour + np.array([[[x0, y0]]], dtype=contour.dtype)
            cv2.drawContours(debug, [contour_shifted], -1, (0, 255, 255), 2)
        elif debug_candidate is not None:
            contour = debug_candidate['contour']
            contour_shifted = contour + np.array([[[x0, y0]]], dtype=contour.dtype)
            cv2.drawContours(debug, [contour_shifted], -1, (0, 180, 255), 2)
        if best_caught is not None:
            contour = best_caught['contour']
            contour_shifted = contour + np.array([[[x0, y0]]], dtype=contour.dtype)
            cv2.drawContours(debug, [contour_shifted], -1, (0, 255, 0), 3)
            cx = int(float(best_caught['cx']) * width)
            cy = int(float(best_caught['cy']) * height)
            cv2.circle(debug, (cx, cy), 4, (0, 255, 0), -1)

        status = 'caught=True' if caught_decision else 'caught=False'
        frame_status = 'frame=True' if state.get('frame_caught') else 'frame=False'
        cv2.putText(debug, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(
            debug,
            f'{frame_status} history={stable_count}/{required}',
            (12, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            debug,
            (
                f'detection_confidence={float(state["detection_confidence"]):.2f} '
                f'caught_score={float(state["caught_score"]):.2f}'
            ),
            (12, 86),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            debug,
            f'mode={self.segmentation_mode}',
            (12, 112),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1,
        )

        metrics = debug_candidate if debug_candidate is not None else best_detection
        if metrics is not None:
            cv2.putText(
                debug,
                (
                    f'area={float(metrics["area_fraction"]):.3f} '
                    f'circ={float(metrics["circularity"]):.2f} '
                    f'cx={float(metrics["cx"]):.2f} cy={float(metrics["cy"]):.2f}'
                ),
                (12, 138),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
            )
            candidate_state = metrics.get('candidate_state', {})
            ok_text = (
                f'area:{"OK" if candidate_state.get("area_ok") else "FAIL"} '
                f'x:{"OK" if candidate_state.get("x_ok") else "FAIL"} '
                f'y:{"OK" if candidate_state.get("y_ok") else "FAIL"}'
            )
            cv2.putText(
                debug,
                ok_text,
                (12, 164),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
            )
            ok_text2 = (
                f'circ:{"OK" if candidate_state.get("circularity_ok") else "FAIL"} '
                f'score:{"OK" if candidate_state.get("score_ok") else "FAIL"}'
            )
            cv2.putText(
                debug,
                ok_text2,
                (12, 190),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
            )
        else:
            cv2.putText(
                debug,
                'no contour after area prefilter',
                (12, 138),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
            )

        mask_canvas = np.zeros_like(frame)
        if mask.size > 0:
            mask_canvas[y0:y1, x0:x1] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(mask_canvas, (x0, y0), (x1, y1), (255, 255, 255), 2)
        cv2.putText(
            mask_canvas,
            'final_mask',
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        return np.hstack((debug, mask_canvas))


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
