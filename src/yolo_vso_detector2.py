#!/usr/bin/env python3
"""
DEPRECATED: Use yolo_vso_detector.py instead.
This file contains an incomplete/abandoned implementation.
Kept for reference only.
"""

import pyzed.sl as sl
import cv2
import numpy as np
import math
import os
from ultralytics import YOLO


class YoloVSODetector2:
    """
    Legacy implementation - incomplete and not maintained.
    Use YOLOVSODetector from yolo_vso_detector.py instead.
    """
    
    def __init__(self, vso_path, yolo_model_path='yolov8n.onnx'):
        # Load YOLO model
        print(f"Loading YOLO model from: {yolo_model_path}")
        self.yolo = YOLO(yolo_model_path)
        print("YOLO model loaded")
        
        # Initialize ZED camera from VSO file
        print(f"Loading VSO file: {vso_path}")
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.set_from_svo_file(vso_path)
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        init_params.sdk_verbose = 1
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Error opening ZED: {err}")
            raise Exception("Failed to open VSO file")
        
        # Get VSO file info
        camera_info = self.zed.get_camera_information()
        self.total_frames = self.zed.get_svo_number_of_frames()
        resolution = camera_info.camera_configuration.resolution
        
        print(f"Loaded VSO: {self.total_frames} frames at {resolution.width}x{resolution.height}")


# Note: This implementation is incomplete. 
# Refer to yolo_vso_detector.py for the full working version.


                











         

