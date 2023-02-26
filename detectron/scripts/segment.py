#!/usr/bin/env python
import rospy
import rospkg
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest

import numpy as np
import os, json, cv2, random, glob

from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.structures import BoxMode
from detectron2.utils.visualizer import ColorMode
from detectron.msg import Detection
from detectron.srv import GetDetections, GetDetectionsResponse


class DetectronROS(object):
    def __init__(self):
        cfg = get_cfg()
        cfg.merge_from_file("/home/stephenalt/catkin_ws/src/frasier_perception/detectron/configs/mask_rcnn_R_50_FPN_3x.yaml")
        thing_classes = ["master_chef_can", "cracker_box", "sugar_box", "tomato_soup_can", "mustard_bottle", "tuna_fish_can", "pudding_box", "gelatin_box", "potted_meat_can", "banana", "pitcher_base", "bleach_cleanser", "bowl", "mug", "power_drill", "wood_block", "scissors", "large_marker", "large_clamp", "extra_large_clamp", "foam_brick"]
        self._thing_classes = thing_classes
        ycb_metadata = MetadataCatalog.get("ycb_train").set(thing_classes=thing_classes)
        cfg.DATASETS.TRAIN = ("ycb_train",)
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = 21
        cfg.MODEL.WEIGHTS = "/home/stephenalt/catkin_ws/src/frasier_perception/detectron/models/model_final.pth"
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.85
        self._cfg = cfg
        self._image = None
        predictor = DefaultPredictor(cfg)
        self._predictor = predictor
        self._cv_bridge = CvBridge()
        sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, self.image_callback, queue_size=1)
        self.detectons_srv = rospy.Service('/mask_rcnn/get_detections', GetDetections, self.detect_service)
        print("ready to segment")

    def image_callback(self, msg):
        np_image = self._cv_bridge.imgmsg_to_cv2(msg, 'rgb8')
        self._image = np_image
        # outputs = self._predictor(np_image)
        # v = Visualizer(np_image[:, :, ::-1], MetadataCatalog.get(self._cfg.DATASETS.TRAIN[0]))
        # out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        # cv2.imshow("window", out.get_image()[:, :, ::-1])
        # cv2.waitKey(10000)
        # Get the fields of the prediction
        # fields = outputs['instances'].get_fields()
        # Get the predicted classes from the fields
        # pred_classes = fields['pred_classes'].cpu().numpy()
        # Number of objects detected in img
        # num_objs = len(pred_classes)
        # Array of the detected objs masks
        # pred_masks = fields['pred_masks'].cpu().numpy()
        # Score of each prediction
        # scores = fields['scores'].cpu().numpy()
        # Put all the data of each obj in the objs array
        # objs = []
        # for i in range(0, num_objs):
            # obj_data = {}
            # obj_data['pred_class'] = pred_classes[i]
            # print(pred_classes[i])
            # pred_mask = pred_masks[i]
            # obj_data['pixels'] = np.where(pred_mask == True)
            # objs.append(obj_data) 
        # print(objs)

    def detect_service(self, req):
        outputs = self._predictor(self._image)
        # v = Visualizer(self._image[:, :, ::-1], MetadataCatalog.get(self._cfg.DATASETS.TRAIN[0]))
        # out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        # cv2.imshow("window", out.get_image()[:, :, ::-1])
        # cv2.waitKey(10000)
        # Get the fields of the prediction
        fields = outputs['instances'].get_fields()
        # Get the predicted classes from the fields
        pred_classes = fields['pred_classes'].cpu().numpy()
        # Bounding boxes
        bounding_boxes = fields['pred_boxes']
        # Number of objects detected in img
        num_objs = len(pred_classes)
        # Array of the detected objs masks
        pred_masks = fields['pred_masks'].cpu().numpy()
        # Score of each prediction
        scores = fields['scores'].cpu().numpy()
        # Put all the data of each obj in the objs array
        objs = []
        for i in range(0, num_objs):
            detection = Detection()
            detection.class_name = self._thing_classes[pred_classes[i]]
            pred_mask = pred_masks[i]
            contours = np.where(pred_mask == True)
            detection.score = scores[i]
            detection.contour_x = contours[1]
            detection.contour_y = contours[0]
            detection.bbox = bounding_boxes[i].tensor.cpu().numpy()[0]
            objs.append(detection)
        return GetDetectionsResponse(objs)

def main():
    rospy.init_node('detectron', anonymous=True)
    node = DetectronROS()
    rospy.spin()

if __name__ == '__main__':
    main()
