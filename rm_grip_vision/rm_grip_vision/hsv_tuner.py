"""Interactive tuner for the EP gripper camera segmentation.

The default mode is for a gray motion-capture-coated tennis ball on a green
field/background.  It shows three masks:
- gray_candidate: low-saturation / near-neutral regions;
- green_background: pixels suppressed as green field/background;
- final_mask: gray_candidate with green_background removed.

Press ``p`` to print ROS parameters that can be copied into the detector config.
"""

from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class HsvTuner(Node):
    def __init__(self) -> None:
        super().__init__('hsv_tuner')
        self.robot_name = str(self.declare_parameter('robot_name', 'RM1').value)
        self.image_topic = str(
            self.declare_parameter('image_topic', f'/{self.robot_name}/camera/image_color').value
        )
        self.roi = [float(v) for v in self.declare_parameter('roi', [0.24, 0.42, 0.76, 0.96]).value]
        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None
        self.create_subscription(Image, self.image_topic, self.image_cb, 5)
        self.get_logger().info(f'gray-on-green tuner subscribed to {self.image_topic}')

    def image_cb(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')


def _nothing(_: int) -> None:
    pass


def _create_controls(window: str) -> None:
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    defaults = {
        'Gray S max': 85,
        'Gray V min': 35,
        'Gray V max': 255,
        'Lab chroma max': 24,
        'Lab L min': 35,
        'Lab L max': 255,
        'Green H min': 35,
        'Green H max': 95,
        'Green S min': 45,
        'Green V min': 35,
        'Kernel': 5,
    }
    for name, value in defaults.items():
        maximum = 179 if name.startswith('Green H') else 255
        if name == 'Lab chroma max':
            maximum = 120
        if name == 'Kernel':
            maximum = 25
        cv2.createTrackbar(name, window, value, maximum, _nothing)


def _read_controls(window: str):
    values = {
        'gray_s_max': cv2.getTrackbarPos('Gray S max', window),
        'gray_v_min': cv2.getTrackbarPos('Gray V min', window),
        'gray_v_max': cv2.getTrackbarPos('Gray V max', window),
        'lab_chroma_max': cv2.getTrackbarPos('Lab chroma max', window),
        'lab_l_min': cv2.getTrackbarPos('Lab L min', window),
        'lab_l_max': cv2.getTrackbarPos('Lab L max', window),
        'green_h_min': cv2.getTrackbarPos('Green H min', window),
        'green_h_max': cv2.getTrackbarPos('Green H max', window),
        'green_s_min': cv2.getTrackbarPos('Green S min', window),
        'green_v_min': cv2.getTrackbarPos('Green V min', window),
        'kernel': max(1, cv2.getTrackbarPos('Kernel', window)),
    }
    if values['kernel'] % 2 == 0:
        values['kernel'] += 1
    return values


def _make_masks(roi_img: np.ndarray, params: dict):
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    hsv_gray = cv2.inRange(
        hsv,
        np.array([0, 0, params['gray_v_min']], dtype=np.uint8),
        np.array([179, params['gray_s_max'], params['gray_v_max']], dtype=np.uint8),
    )

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

    gray_candidate = cv2.bitwise_or(hsv_gray, lab_gray)
    green_background = cv2.inRange(
        hsv,
        np.array([params['green_h_min'], params['green_s_min'], params['green_v_min']], dtype=np.uint8),
        np.array([params['green_h_max'], 255, 255], dtype=np.uint8),
    )
    final_mask = cv2.bitwise_and(gray_candidate, cv2.bitwise_not(green_background))

    kernel = np.ones((params['kernel'], params['kernel']), dtype=np.uint8)
    final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
    final_mask = cv2.morphologyEx(final_mask, cv2.roiMORPH_CLOSE, kernel)
    return gray_candidate, green_background, final_mask


def _print_params(params: dict, roi) -> None:
    print('\nCopy these parameters into your launch command/config:')
    print(
        'ros2 run rm_grip_vision grip_vision_detector --ros-args '
        '-p segmentation_mode:=gray_on_green '
        f'-p gray_s_max:={params["gray_s_max"]} '
        f'-p gray_v_min:={params["gray_v_min"]} '
        f'-p gray_v_max:={params["gray_v_max"]} '
        f'-p lab_chroma_max:={params["lab_chroma_max"]} '
        f'-p lab_l_min:={params["lab_l_min"]} '
        f'-p lab_l_max:={params["lab_l_max"]} '
        f'-p green_h_min:={params["green_h_min"]} '
        f'-p green_h_max:={params["green_h_max"]} '
        f'-p green_s_min:={params["green_s_min"]} '
        f'-p green_v_min:={params["green_v_min"]} '
        f'-p roi:="[{roi[0]}, {roi[1]}, {roi[2]}, {roi[3]}]"'
    )
    print('YAML:')
    print('  segmentation_mode: gray_on_green')
    for key in [
        'gray_s_max', 'gray_v_min', 'gray_v_max', 'lab_chroma_max', 'lab_l_min',
        'lab_l_max', 'green_h_min', 'green_h_max', 'green_s_min', 'green_v_min'
    ]:
        print(f'  {key}: {params[key]}')
    print(f'  roi: [{roi[0]}, {roi[1]}, {roi[2]}, {roi[3]}]')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HsvTuner()
    window = 'gray_on_green_tuner'
    _create_controls(window)

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
            x0 = int(node.roi[0] * width)
            y0 = int(node.roi[1] * height)
            x1 = int(node.roi[2] * width)
            y1 = int(node.roi[3] * height)
            roi_img = frame[y0:y1, x0:x1]

            params = _read_controls(window)
            gray_candidate, green_background, final_mask = _make_masks(roi_img, params)
            masked = cv2.bitwise_and(roi_img, roi_img, mask=final_mask)

            debug = frame.copy()
            cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 255), 2)
            cv2.putText(
                debug,
                'p: print params, q: quit',
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.imshow('camera_with_roi', debug)
            cv2.imshow(window, masked)
            cv2.imshow('gray_candidate', gray_candidate)
            cv2.imshow('green_background_suppressed', green_background)
            cv2.imshow('final_mask', final_mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('p'):
                _print_params(params, node.roi)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
