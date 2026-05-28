"""Backward-compatible entry point for the old hsv_tuner command.

New users should run:
    ros2 run rm_grip_vision segmentation_tuner
"""

from rm_grip_vision.segmentation_tuner import main


if __name__ == '__main__':
    main()
