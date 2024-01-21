# coding=utf-8
import multiprocessing
import queue

import math

from scipy.spatial.transform import Rotation as R
import pandas as pd
import cv2, os, yaml
from cv2 import aruco

import socket, struct, time
import matplotlib.pyplot as plt
import numpy as np


class detector(object):
    def __init__(self, source='./'):
        super().__init__()
        self.source = source
        self.config = yaml.load(open(dir + '/controller_config.yaml'), Loader=yaml.FullLoader)
        self.config = self.config['vision']
        self._run_flag = True
        self.camera_matrix = np.matrix(self.config['camera_matrix'])
        self.camera_dist = np.matrix(self.config['camera_dist'])
        self.marker_length = self.config['marker_length']
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)

        self.parameters = aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        self.tag_pose_vecs = np.array([[0, 0, 0, 0, 0, 0]])
        self.force_vecs = np.array([[0, 0, 0, 0, 0, 0]])
        self.detection_failure = []
        self.time_cost = []

    def detect(self, color_image):
        # capture from web cam

        gray_ = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((5, 5), np.float32) / 25
        gray = cv2.filter2D(gray_, -1, kernel)

        current_corners, current_ids, _ = aruco.detectMarkers(gray,
                                                              self.aruco_dict,
                                                              parameters=self.parameters)
        if current_ids is None:
            return None

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(current_corners,
                                                          self.marker_length,
                                                          self.camera_matrix,
                                                          self.camera_dist)
        rr = R.from_rotvec(rvecs[0])
        # matrix = rr.as_matrix()
        # rtvecs = rr.apply(tvecs[0][0] * 1000)
        rpy = rr.as_euler('xyz', degrees=True)[0]
        rpy[-1] = rpy[-1] + 360 * (rpy[-1] < 0)
        # rpy[0] = rpy[0] + 360 * (rpy[0] < 0)

        return np.concatenate((tvecs[0][0] * 1000, rpy))


class GetFingerForce(object):
    def __init__(self, dataDir):
        self.dataDir = dataDir
        # Camera ready
        for i in range(1, 8):
            self.cap = cv2.VideoCapture(i)
            if self.cap.isOpened():
                print('camera open')
                ret, img = self.cap.read()
                cv2.imshow('2', img)
                cv2.waitKey(1000)
                break
        if not self.cap.isOpened():
            exit()
        self.cap.set(cv2.CAP_PROP_FPS, 120)
        cv2.destroyAllWindows()
        self.detect = detector()

    def read_pose(self):
        finger_pose = None
        while finger_pose is None:
            ret, img = self.cap.read()
            if not ret:
                continue
            finger_pose = self.detect.detect(color_image=img)
        return finger_pose
