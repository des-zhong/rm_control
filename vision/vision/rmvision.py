import cv2
import robomaster
from robomaster import robot
from robomaster import vision
from ultralytics import YOLO
import time
# from .fastsam import FastSAM, FastSAMPrompt
import os

from rclpy.node import Node
import rclpy


class Rmvision(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name=name
        self.id=int(self.get_name()[3])
        self.model = YOLO('./src/rmyscontrol/vision/vision/weights/yolov8n.pt')
        # self.model_fsam = FastSAM('/home/zhe/code/FastSAM/weights/FastSAM.pt')
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta")

        self.ep_vision = self.ep_robot.vision
        self.ep_camera = self.ep_robot.camera

        self.ep_camera.start_video_stream(display=False)
        self.ep_vision.sub_detect_info(name="robot", callback=self.on_detect_person)
        self.robots = []
    
    def on_detect_person(self, person_info):
        number = len(person_info)
        self.robots.clear()
        for i in range(0, number):
            x, y, w, h = person_info[i]
            self.robots.append(RobotInfo(x, y, w, h))
            # print("robot: x:{0}, y:{1}, w:{2}, h:{3}".format(x, y, w, h))
    
        


class RobotInfo:

    def __init__(self, x, y, w, h):
        self._x = x
        self._y = y
        self._w = w
        self._h = h

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)
    




def predict(chosen_model, img, classes=[], conf=0.5):
    if classes:
        results = chosen_model.predict(img, classes=classes, conf=conf)
    else:
        results = chosen_model.predict(img, conf=conf)

    return results


def predict_and_detect(chosen_model, img, classes=[], conf=0.5):
    results = predict(chosen_model, img, classes, conf=conf)

    for result in results:
        for box in result.boxes:
            cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                          (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), 2)
            cv2.putText(img, f"{result.names[int(box.cls[0])]}",
                        (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)
    return img, results


def main(args=None):
    rclpy.init(args=args)
    node = Rmvision(name='RMV1')
    print(f'vision node{node.id} started')
    while rclpy.ok():
        img = node.ep_camera.read_cv2_image(strategy="newest", timeout=5)
        ## 原始
        for j in range(0, len(node.robots)):
            cv2.rectangle(img, node.robots[j].pt1, node.robots[j].pt2, (255, 255, 255))
        cv2.imshow("robots", img)
        cv2.waitKey(1)
        # yolo
        # result_img, _ = predict_and_detect(model, img, classes=[], conf=0.5)
        # cv2.imshow('image', img)
        # cv2.waitKey(100)
        
    node.destroy_node()
    rclpy.shutdown()