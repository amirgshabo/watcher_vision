#!/usr/bin/env python3
"""
YOLO Object Detection on ZED VSO Files
Combines YOLO 2D detection with ZED 3D depth for real-world localization
"""

import pyzed.sl as sl
from ultralytics import YOLO
import cv2
import numpy as np
import math
import os

class YOLOVSODetector:
    def __init__(self, vso_path, yolo_model_path='yolov8n.onnx'):
        """
        Initialize YOLO model and ZED camera from VSO file
        
        Args:
            vso_path: Path to .svo video file
            yolo_model_path: Path to ONNX model file
        """
        
        # Load YOLO model
        print(f"Loading YOLO model from: {yolo_model_path}")
        self.yolo = YOLO(yolo_model_path)
        print(f"YOLO model loaded successfully")
        
        # Setup ZED camera from VSO
        print(f"\nLoading VSO file: {vso_path}")
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.set_from_svo_file(vso_path)
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        init_params.sdk_verbose = 1
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Error opening VSO file: {err}")
            raise Exception("Failed to open VSO file")
        
        # Get camera info
        camera_info = self.zed.get_camera_information()
        self.total_frames = self.zed.get_svo_number_of_frames()
        resolution = camera_info.camera_configuration.resolution
        
        print(f"VSO file loaded successfully")
        print(f"   Total frames: {self.total_frames}")
        print(f"   Resolution: {resolution.width}x{resolution.height}")
        print(f"   Depth mode: NEURAL")
    
    def detect_objects_in_frame(self, image_mat, depth_mat, point_cloud_mat):
        """
        Run YOLO detection on frame and add 3D coordinates from depth
        
        Returns: List of detections with 2D + 3D information
        """
        # Get RGB image from ZED
        image_np = image_mat.get_data()
        image_bgr = cv2.cvtColor(image_np, cv2.COLOR_RGBA2BGR)
        
        # Run YOLO detection (returns 2D bounding boxes + class labels)
        results = self.yolo(image_bgr)
        
        detections = []
        
        # Process each detection
        for detection in results[0].boxes:
            # Extract 2D bounding box coordinates
            x1, y1, x2, y2 = detection.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Get class name and confidence
            class_id = int(detection.cls[0])
            confidence = float(detection.conf[0])
            class_name = results[0].names[class_id]
            
            # Calculate center of bounding box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Get 3D coordinates from point cloud at detection center
            err, point_3d = point_cloud_mat.get_value(center_x, center_y)
            
            if err == sl.ERROR_CODE.SUCCESS:
                # Calculate 3D distance using Euclidean distance
                distance_3d = math.sqrt(
                    point_3d[0]**2 + point_3d[1]**2 + point_3d[2]**2
                )
                
                detection_info = {
                    'class': class_name,
                    'confidence': confidence,
                    'bbox_2d': (x1, y1, x2, y2),
                    'center_2d': (center_x, center_y),
                    '3d_coords': point_3d,
                    '3d_distance': distance_3d,
                    'depth_z': point_3d[2]  # Z-depth only
                }
                
                detections.append(detection_info)
        
        return detections, image_bgr
    
    def draw_detections(self, image, detections):
        """
        Draw bounding boxes and 3D info on image
        """
        for det in detections:
            x1, y1, x2, y2 = det['bbox_2d']
            center_x, center_y = det['center_2d']
            
            # Draw bounding box (green rectangle)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw class label with confidence
            label = f"{det['class']} ({det['confidence']:.2f})"
            cv2.putText(image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw 3D distance info
            distance_text = f"3D: {det['3d_distance']:.2f}m"
            cv2.putText(image, distance_text, (x1, y2 + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Draw center point
            cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        return image
    
    def process_vso(self, max_frames=500, start_frame=0, yolo_every=3):
        """
        Process VSO file frame-by-frame with YOLO detection.
        Runs YOLO every `yolo_every` frames; all frames are displayed
        with the last known detections overlaid for smooth playback.
        """
        print(f"\n{'='*60}")
        print(f"Processing {max_frames} frames with YOLO detection...")
        print(f"Starting from frame {start_frame}")
        print(f"YOLO inference every {yolo_every} frame(s)")
        print(f"{'='*60}\n")
        
        # Skip to start frame
        if start_frame > 0 and start_frame < self.total_frames:
            self.zed.set_svo_position(start_frame)
            print(f"Skipped to frame {start_frame}\n")
        
        image = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()
        runtime_params = sl.RuntimeParameters()
        
        frame_count = 0
        last_detections = []  # reuse detections on non-YOLO frames
        detection_stats = {
            'total_frames': 0,
            'frames_with_detections': 0,
            'total_objects': 0,
            'objects_by_class': {}
        }
        
        while frame_count < max_frames and (start_frame + frame_count) < self.total_frames:
            # Grab frame from VSO
            if self.zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
                break
            
            # Always retrieve the RGB image for display
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            image_np = image.get_data()
            image_bgr = cv2.cvtColor(image_np, cv2.COLOR_RGBA2BGR)

            # Run YOLO only every `yolo_every` frames
            if frame_count % yolo_every == 0:
                self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                detections, _ = self.detect_objects_in_frame(image, depth, point_cloud)
                last_detections = detections

                # Update statistics
                detection_stats['total_frames'] += 1
                if detections:
                    detection_stats['frames_with_detections'] += 1
                    detection_stats['total_objects'] += len(detections)
                    for det in detections:
                        class_name = det['class']
                        if class_name not in detection_stats['objects_by_class']:
                            detection_stats['objects_by_class'][class_name] = 0
                        detection_stats['objects_by_class'][class_name] += 1

                # Print results every 30 frames
                if frame_count % 30 == 0:
                    frame_num = start_frame + frame_count
                    print(f"Frame {frame_num}:")
                    if detections:
                        for det in detections:
                            print(f"  - {det['class']}: confidence={det['confidence']:.2f}, "
                                  f"3D distance={det['3d_distance']:.2f}m, "
                                  f"Z-depth={det['depth_z']:.2f}m")
                    else:
                        print("  - No objects detected")
            
            # Draw last known detections on every frame
            display_image = self.draw_detections(image_bgr.copy(), last_detections)
            
            # Display results
            cv2.imshow('YOLO Detection on ZED VSO', display_image)
            
            # ~30 fps playback: wait 33ms, check for key presses
            key = cv2.waitKey(33) & 0xFF
            if key == ord('q'):
                print("\nUser stopped processing")
                break
            elif key == ord(' '):
                print("\n⏸️  Paused - press any key to continue")
                cv2.waitKey(0)
            
            frame_count += 1
        
        # Cleanup
        self.zed.close()
        cv2.destroyAllWindows()
        
        # Print summary statistics
        print(f"\nDETECTION SUMMARY")
        print(f"Frames processed: {detection_stats['total_frames']}")
        print(f"Frames with detections: {detection_stats['frames_with_detections']} "
              f"({100*detection_stats['frames_with_detections']/max(detection_stats['total_frames'], 1):.1f}%)")
        print(f"Total objects detected: {detection_stats['total_objects']}")
        print(f"\nObjects by class:")
        for class_name, count in sorted(detection_stats['objects_by_class'].items(), 
                                       key=lambda x: x[1], reverse=True):
            print(f"  - {class_name}: {count} detections")
        print(f"\nProcessing complete")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='YOLO Object Detection on ZED VSO Files')
    parser.add_argument('--input', '-i', required=True, help='Path to VSO file')
    parser.add_argument('--model', '-m', default='yolov8n.onnx', help='Path to ONNX model')
    parser.add_argument('--frames', '-f', type=int, default=500, help='Number of frames to process')
    parser.add_argument('--start', '-s', type=int, default=0, help='Start frame number')
    parser.add_argument('--yolo-every', '-y', type=int, default=3, help='Run YOLO every N frames (higher = smoother)')
    
    args = parser.parse_args()
    
    # Check if VSO file exists
    if not os.path.exists(args.input):
        print(f"Error: VSO file not found: {args.input}")
        return 1
    
    # Check if model exists
    if not os.path.exists(args.model):
        print(f"Error: ONNX model not found: {args.model}")
        return 1
    
    print("\n" + "="*60)
    print("YOLO Object Detection on ZED VSO Files")
    print("="*60)
    
    try:
        detector = YOLOVSODetector(args.input, args.model)
        detector.process_vso(args.frames, args.start, args.yolo_every)
        return 0
    except Exception as e:
        print(f"Error: {e}")
        return 1

if __name__ == "__main__":
    exit(main())
