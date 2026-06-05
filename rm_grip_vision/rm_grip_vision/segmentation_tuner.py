"""Interactive tuner for EP gripper-camera segmentation and ROI.

OpenCV/cv_bridge 默认使用 BGR 图像顺序。调 BGR 阈值时直接使用原图
[B, G, R]；调 RGB 阈值时程序会先把 ROI 从 BGR 转成 RGB。

Keys:
- p: print YAML parameters
- q: quit
"""

from typing import Dict, Iterable, List, Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


MODE_NAMES = ['gray_on_green', 'bgr_ranges', 'rgb_ranges', 'hsv_ranges']
ROI_SCALE = 1000
ROI_MIN_SPAN = 0.02


def _as_int_list(value: Iterable[int], *, length: int, name: str) -> List[int]:
    values = [int(v) for v in value]
    if len(values) != length:
        raise ValueError(f'{name} must contain exactly {length} integers, got {values}')
    return [max(0, min(255, v)) for v in values]


def _mode_index(name: str) -> int:
    try:
        return MODE_NAMES.index(str(name).lower())
    except ValueError:
        return 0


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _sanitize_roi(roi) -> List[float]:
    """裁剪并修正 ROI，保证至少保留 2% 的宽和高。"""
    x_min, y_min, x_max, y_max = [_clamp01(v) for v in roi]

    if x_max - x_min < ROI_MIN_SPAN:
        if x_min <= 1.0 - ROI_MIN_SPAN:
            x_max = x_min + ROI_MIN_SPAN
        else:
            x_min = max(0.0, x_max - ROI_MIN_SPAN)

    if y_max - y_min < ROI_MIN_SPAN:
        if y_min <= 1.0 - ROI_MIN_SPAN:
            y_max = y_min + ROI_MIN_SPAN
        else:
            y_min = max(0.0, y_max - ROI_MIN_SPAN)

    if x_min >= x_max:
        x_min = max(0.0, x_max - ROI_MIN_SPAN)
    if y_min >= y_max:
        y_min = max(0.0, y_max - ROI_MIN_SPAN)

    return [x_min, y_min, x_max, y_max]


class SegmentationTuner(Node):
    def __init__(self) -> None:
        super().__init__('segmentation_tuner')
        self.robot_name = str(self.declare_parameter('robot_name', 'RM1').value)
        self.image_topic = str(
            self.declare_parameter('image_topic', f'/{self.robot_name}/camera/image_color').value
        )
        self.roi = _sanitize_roi(
            [float(v) for v in self.declare_parameter('roi', [0.24, 0.42, 0.76, 0.96]).value]
        )
        self.defaults = self.read_initial_parameters()
        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None
        self.create_subscription(Image, self.image_topic, self.image_cb, 5)
        self.get_logger().info(
            f'分割调参器已订阅 {self.image_topic}; '
            f'mode={self.defaults["segmentation_mode"]}, roi={self.roi}'
        )

    def read_initial_parameters(self) -> Dict[str, object]:
        segmentation_mode = str(
            self.declare_parameter('segmentation_mode', 'gray_on_green').value
        ).lower()
        bgr_lower = _as_int_list(
            self.declare_parameter('bgr_lower', [0, 0, 0]).value,
            length=3,
            name='bgr_lower',
        )
        bgr_upper = _as_int_list(
            self.declare_parameter('bgr_upper', [255, 255, 255]).value,
            length=3,
            name='bgr_upper',
        )
        rgb_lower = _as_int_list(
            self.declare_parameter('rgb_lower', [0, 0, 0]).value,
            length=3,
            name='rgb_lower',
        )
        rgb_upper = _as_int_list(
            self.declare_parameter('rgb_upper', [255, 255, 255]).value,
            length=3,
            name='rgb_upper',
        )
        if segmentation_mode == 'rgb_ranges':
            b_min, g_min, r_min = rgb_lower[2], rgb_lower[1], rgb_lower[0]
            b_max, g_max, r_max = rgb_upper[2], rgb_upper[1], rgb_upper[0]
        else:
            b_min, g_min, r_min = bgr_lower
            b_max, g_max, r_max = bgr_upper

        hsv_lower = _as_int_list(
            self.declare_parameter('hsv_lower', [0, 70, 60]).value,
            length=3,
            name='hsv_lower',
        )
        hsv_upper = _as_int_list(
            self.declare_parameter('hsv_upper', [28, 255, 255]).value,
            length=3,
            name='hsv_upper',
        )
        return {
            'segmentation_mode': segmentation_mode,
            'mode_index': _mode_index(segmentation_mode),
            'b_min': b_min,
            'b_max': b_max,
            'g_min': g_min,
            'g_max': g_max,
            'r_min': r_min,
            'r_max': r_max,
            'gray_s_max': int(self.declare_parameter('gray_s_max', 85).value),
            'gray_v_min': int(self.declare_parameter('gray_v_min', 35).value),
            'gray_v_max': int(self.declare_parameter('gray_v_max', 255).value),
            'use_lab_gray': int(bool(self.declare_parameter('use_lab_gray', True).value)),
            'lab_chroma_max': int(float(self.declare_parameter('lab_chroma_max', 24.0).value)),
            'lab_l_min': int(self.declare_parameter('lab_l_min', 35).value),
            'lab_l_max': int(self.declare_parameter('lab_l_max', 255).value),
            'use_green_suppression': int(
                bool(self.declare_parameter('use_green_suppression', True).value)
            ),
            'green_h_min': int(self.declare_parameter('green_h_min', 35).value),
            'green_h_max': int(self.declare_parameter('green_h_max', 95).value),
            'green_s_min': int(self.declare_parameter('green_s_min', 45).value),
            'green_v_min': int(self.declare_parameter('green_v_min', 35).value),
            'h_min': hsv_lower[0],
            's_min': hsv_lower[1],
            'v_min': hsv_lower[2],
            'h_max': hsv_upper[0],
            's_max': hsv_upper[1],
            'v_max': hsv_upper[2],
            'kernel': int(self.declare_parameter('morph_kernel', 5).value),
        }

    def image_cb(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')


def _nothing(_: int) -> None:
    pass


def _create_roi_controls(window: str, roi) -> None:
    """ROI 用 0-1000 整数滑条表示 0.0-1.0 归一化范围。"""
    roi = _sanitize_roi(roi)
    names = ['ROI x_min', 'ROI y_min', 'ROI x_max', 'ROI y_max']
    for name, value in zip(names, roi):
        cv2.createTrackbar(name, window, int(value * ROI_SCALE), ROI_SCALE, _nothing)


def _read_roi_from_trackbars(window: str) -> List[float]:
    roi = [
        cv2.getTrackbarPos('ROI x_min', window) / ROI_SCALE,
        cv2.getTrackbarPos('ROI y_min', window) / ROI_SCALE,
        cv2.getTrackbarPos('ROI x_max', window) / ROI_SCALE,
        cv2.getTrackbarPos('ROI y_max', window) / ROI_SCALE,
    ]
    sanitized = _sanitize_roi(roi)
    for name, value in zip(['ROI x_min', 'ROI y_min', 'ROI x_max', 'ROI y_max'], sanitized):
        slider_value = int(round(value * ROI_SCALE))
        if cv2.getTrackbarPos(name, window) != slider_value:
            cv2.setTrackbarPos(name, window, slider_value)
    return sanitized


def _create_controls(window: str, defaults: Dict[str, object]) -> None:
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    controls = [
        ('Mode', int(defaults['mode_index']), len(MODE_NAMES) - 1),
        ('B min', int(defaults['b_min']), 255),
        ('B max', int(defaults['b_max']), 255),
        ('G min', int(defaults['g_min']), 255),
        ('G max', int(defaults['g_max']), 255),
        ('R min', int(defaults['r_min']), 255),
        ('R max', int(defaults['r_max']), 255),
        ('Gray S max', int(defaults['gray_s_max']), 255),
        ('Gray V min', int(defaults['gray_v_min']), 255),
        ('Gray V max', int(defaults['gray_v_max']), 255),
        ('Use Lab Gray', int(defaults['use_lab_gray']), 1),
        ('Lab chroma max', int(defaults['lab_chroma_max']), 120),
        ('Lab L min', int(defaults['lab_l_min']), 255),
        ('Lab L max', int(defaults['lab_l_max']), 255),
        ('Green Suppress', int(defaults['use_green_suppression']), 1),
        ('Green H min', int(defaults['green_h_min']), 179),
        ('Green H max', int(defaults['green_h_max']), 179),
        ('Green S min', int(defaults['green_s_min']), 255),
        ('Green V min', int(defaults['green_v_min']), 255),
        ('HSV H min', int(defaults['h_min']), 179),
        ('HSV H max', int(defaults['h_max']), 179),
        ('HSV S min', int(defaults['s_min']), 255),
        ('HSV S max', int(defaults['s_max']), 255),
        ('HSV V min', int(defaults['v_min']), 255),
        ('HSV V max', int(defaults['v_max']), 255),
        ('Kernel', max(1, int(defaults['kernel'])), 25),
    ]
    for name, value, maximum in controls:
        cv2.createTrackbar(name, window, value, maximum, _nothing)


def _read_controls(window: str) -> Dict[str, object]:
    mode_index = cv2.getTrackbarPos('Mode', window)
    mode_index = max(0, min(len(MODE_NAMES) - 1, mode_index))
    values = {
        'segmentation_mode': MODE_NAMES[mode_index],
        'b_min': cv2.getTrackbarPos('B min', window),
        'b_max': cv2.getTrackbarPos('B max', window),
        'g_min': cv2.getTrackbarPos('G min', window),
        'g_max': cv2.getTrackbarPos('G max', window),
        'r_min': cv2.getTrackbarPos('R min', window),
        'r_max': cv2.getTrackbarPos('R max', window),
        'gray_s_max': cv2.getTrackbarPos('Gray S max', window),
        'gray_v_min': cv2.getTrackbarPos('Gray V min', window),
        'gray_v_max': cv2.getTrackbarPos('Gray V max', window),
        'use_lab_gray': bool(cv2.getTrackbarPos('Use Lab Gray', window)),
        'lab_chroma_max': cv2.getTrackbarPos('Lab chroma max', window),
        'lab_l_min': cv2.getTrackbarPos('Lab L min', window),
        'lab_l_max': cv2.getTrackbarPos('Lab L max', window),
        'use_green_suppression': bool(cv2.getTrackbarPos('Green Suppress', window)),
        'green_h_min': cv2.getTrackbarPos('Green H min', window),
        'green_h_max': cv2.getTrackbarPos('Green H max', window),
        'green_s_min': cv2.getTrackbarPos('Green S min', window),
        'green_v_min': cv2.getTrackbarPos('Green V min', window),
        'h_min': cv2.getTrackbarPos('HSV H min', window),
        'h_max': cv2.getTrackbarPos('HSV H max', window),
        's_min': cv2.getTrackbarPos('HSV S min', window),
        's_max': cv2.getTrackbarPos('HSV S max', window),
        'v_min': cv2.getTrackbarPos('HSV V min', window),
        'v_max': cv2.getTrackbarPos('HSV V max', window),
        'kernel': max(1, cv2.getTrackbarPos('Kernel', window)),
    }
    if values['kernel'] % 2 == 0:
        values['kernel'] += 1
    return values


def _green_mask(hsv: np.ndarray, params: Dict[str, object]) -> np.ndarray:
    if not params['use_green_suppression']:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)
    return cv2.inRange(
        hsv,
        np.array(
            [params['green_h_min'], params['green_s_min'], params['green_v_min']],
            dtype=np.uint8,
        ),
        np.array([params['green_h_max'], 255, 255], dtype=np.uint8),
    )


def _make_gray_candidate(roi_img: np.ndarray, params: Dict[str, object]) -> np.ndarray:
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    hsv_gray = cv2.inRange(
        hsv,
        np.array([0, 0, params['gray_v_min']], dtype=np.uint8),
        np.array([179, params['gray_s_max'], params['gray_v_max']], dtype=np.uint8),
    )
    if not params['use_lab_gray']:
        return hsv_gray

    lab = cv2.cvtColor(roi_img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    chroma = np.sqrt(
        (a.astype(np.float32) - 128.0) ** 2 +
        (b.astype(np.float32) - 128.0) ** 2
    )
    lab_gray = (
        (chroma <= float(params['lab_chroma_max'])) &
        (l >= params['lab_l_min']) &
        (l <= params['lab_l_max'])
    ).astype(np.uint8) * 255
    return cv2.bitwise_or(hsv_gray, lab_gray)


def _make_masks(roi_img: np.ndarray, params: Dict[str, object]):
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    mode = str(params['segmentation_mode'])

    # 所有模式只处理当前 ROI，避免调参时 mask 和检测范围不一致。
    if mode == 'bgr_ranges':
        candidate = cv2.inRange(
            roi_img,
            np.array([params['b_min'], params['g_min'], params['r_min']], dtype=np.uint8),
            np.array([params['b_max'], params['g_max'], params['r_max']], dtype=np.uint8),
        )
        candidate_label = 'bgr_candidate'
    elif mode == 'rgb_ranges':
        roi_rgb = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
        candidate = cv2.inRange(
            roi_rgb,
            np.array([params['r_min'], params['g_min'], params['b_min']], dtype=np.uint8),
            np.array([params['r_max'], params['g_max'], params['b_max']], dtype=np.uint8),
        )
        candidate_label = 'rgb_candidate'
    elif mode == 'hsv_ranges':
        candidate = cv2.inRange(
            hsv,
            np.array([params['h_min'], params['s_min'], params['v_min']], dtype=np.uint8),
            np.array([params['h_max'], params['s_max'], params['v_max']], dtype=np.uint8),
        )
        candidate_label = 'hsv_candidate'
    else:
        candidate = _make_gray_candidate(roi_img, params)
        candidate_label = 'gray_candidate'

    green_background = _green_mask(hsv, params)
    if params['use_green_suppression']:
        final_mask = cv2.bitwise_and(candidate, cv2.bitwise_not(green_background))
    else:
        final_mask = candidate

    kernel = np.ones((int(params['kernel']), int(params['kernel'])), dtype=np.uint8)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)
    return candidate_label, candidate, green_background, final_mask


def _print_params(params: Dict[str, object], roi) -> None:
    mode = str(params['segmentation_mode'])
    roi_text = ', '.join(f'{v:.3f}' for v in roi)
    print('\nCopy these parameters into config/grip_vision_default.yaml:')
    print('/**:')
    print('  ros__parameters:')
    print(f'    segmentation_mode: {mode}')
    print(f'    roi: [{roi_text}]')
    print(f'    morph_kernel: {params["kernel"]}')
    print(f'    use_green_suppression: {str(params["use_green_suppression"]).lower()}')
    print(f'    green_h_min: {params["green_h_min"]}')
    print(f'    green_h_max: {params["green_h_max"]}')
    print(f'    green_s_min: {params["green_s_min"]}')
    print(f'    green_v_min: {params["green_v_min"]}')

    if mode == 'bgr_ranges':
        print(f'    bgr_lower: [{params["b_min"]}, {params["g_min"]}, {params["r_min"]}]')
        print(f'    bgr_upper: [{params["b_max"]}, {params["g_max"]}, {params["r_max"]}]')
    elif mode == 'rgb_ranges':
        print(f'    rgb_lower: [{params["r_min"]}, {params["g_min"]}, {params["b_min"]}]')
        print(f'    rgb_upper: [{params["r_max"]}, {params["g_max"]}, {params["b_max"]}]')
    elif mode == 'hsv_ranges':
        print(f'    hsv_lower: [{params["h_min"]}, {params["s_min"]}, {params["v_min"]}]')
        print(f'    hsv_upper: [{params["h_max"]}, {params["s_max"]}, {params["v_max"]}]')
    else:
        print(f'    gray_s_max: {params["gray_s_max"]}')
        print(f'    gray_v_min: {params["gray_v_min"]}')
        print(f'    gray_v_max: {params["gray_v_max"]}')
        print(f'    use_lab_gray: {str(params["use_lab_gray"]).lower()}')
        print(f'    lab_chroma_max: {params["lab_chroma_max"]}')
        print(f'    lab_l_min: {params["lab_l_min"]}')
        print(f'    lab_l_max: {params["lab_l_max"]}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SegmentationTuner()
    window = 'segmentation_tuner'
    _create_controls(window, node.defaults)
    _create_roi_controls(window, node.roi)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            frame = node.latest_frame
            if frame is None:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                continue

            height, width = frame.shape[:2]
            current_roi = _read_roi_from_trackbars(window)
            x0 = int(current_roi[0] * width)
            y0 = int(current_roi[1] * height)
            x1 = min(width, max(x0 + 1, int(current_roi[2] * width)))
            y1 = min(height, max(y0 + 1, int(current_roi[3] * height)))
            roi_img = frame[y0:y1, x0:x1]

            params = _read_controls(window)
            candidate_label, candidate, green_background, final_mask = _make_masks(roi_img, params)
            masked = cv2.bitwise_and(roi_img, roi_img, mask=final_mask)

            debug = frame.copy()
            cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.putText(
                debug,
                f'mode={params["segmentation_mode"]}  p: print  q: quit',
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                debug,
                f'roi=[{current_roi[0]:.3f}, {current_roi[1]:.3f}, '
                f'{current_roi[2]:.3f}, {current_roi[3]:.3f}]',
                (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )
            cv2.imshow('camera_with_roi', debug)
            cv2.imshow('original_roi_bgr', roi_img)
            cv2.imshow('segmented_roi', masked)
            cv2.imshow(candidate_label, candidate)
            cv2.imshow('green_background_suppressed', green_background)
            cv2.imshow('final_mask', final_mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('p'):
                _print_params(params, current_roi)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
