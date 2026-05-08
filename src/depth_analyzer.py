#!/usr/bin/env python3
"""
ZED SDK Depth Analysis Python Script
Analyzes VSO files and extracts depth information using Python ZED SDK
"""

import pyzed.sl as sl
import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from pathlib import Path
import json
from tqdm import tqdm
import math
import time

class DepthAnalyzerPython:
    def __init__(self, output_dir="output"):
        self.output_dir = Path(output_dir)
        self.create_output_directories()
        self.depth_stats = []
        
        # Detection parameters (from ROS2 depth subscriber)
        self.min_depth = 0.1          # Minimum depth in meters
        self.max_depth = 10.0         # Maximum depth in meters  
        self.obstacle_threshold = 1.0  # Objects closer than 1m are "obstacles"
        
        # Center point analysis (StereoLabs style)
        self.center_distances = []
        
    def create_output_directories(self):
        """Create necessary output directories"""
        self.output_dir.mkdir(exist_ok=True)
        (self.output_dir / "depth_maps").mkdir(exist_ok=True)
        (self.output_dir / "point_clouds").mkdir(exist_ok=True)
        (self.output_dir / "analysis").mkdir(exist_ok=True)
        
    def process_vso_file(self, vso_path, max_frames=None):
        """Process a VSO file and extract depth information"""
        print(f"Processing VSO file: {vso_path}")
        
        # Initialize ZED camera
        zed = sl.Camera()
        
        # Set initialization parameters (StereoLabs pattern)
        init_params = sl.InitParameters()
        init_params.set_from_svo_file(vso_path)
        init_params.sdk_verbose = True  # Enable verbose logging
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Set to performance (fastest)
        init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units (StereoLabs default)
        
        # Open the camera
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Error opening ZED camera: {err}")
            return False
            
        # Get camera information
        camera_info = zed.get_camera_information()
        total_frames = zed.get_svo_number_of_frames()
        
        print(f"Total frames: {total_frames}")
        print(f"Resolution: {camera_info.camera_resolution.width}x{camera_info.camera_resolution.height}")
        print(f"FPS: {camera_info.camera_fps}")
        
        # Limit frames if specified
        frames_to_process = min(total_frames, max_frames) if max_frames else total_frames
        
        # Initialize matrices (StereoLabs pattern)
        image = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()
        
        # Process frames
        runtime_parameters = sl.RuntimeParameters()
        
        with tqdm(total=frames_to_process, desc="Processing frames") as pbar:
            frame_count = 0
            
            while frame_count < frames_to_process:
                # Grab frame (StereoLabs pattern)
                if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                    # A new image is available if grab() returns sl.ERROR_CODE.SUCCESS
                    zed.retrieve_image(image, sl.VIEW.LEFT)  # Get the left image
                    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)  # Retrieve depth matrix. Depth is aligned on the left RGB image
                    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)  # Retrieve colored point cloud
                    
                    # Process frame with ROS2-style analysis
                    self.process_frame_advanced(depth, point_cloud, image, frame_count)
                    
                    # Save depth map every 10 frames
                    if frame_count % 10 == 0:
                        self.save_depth_map(depth, frame_count)
                    
                    # Save point cloud every 50 frames
                    if frame_count % 50 == 0:
                        self.save_point_cloud(point_cloud, frame_count)
                    
                    frame_count += 1
                    pbar.update(1)
                    
                else:
                    print("Failed to grab frame")
                    break
        
        # Close camera
        zed.close()
        
        # Generate analysis report
        self.generate_analysis_report()
        
        return True
    
    def process_frame(self, depth_mat, frame_number):
        """Process a single depth frame and extract statistics"""
        # Convert ZED Mat to numpy array
        depth_np = depth_mat.get_data()
        
        # Remove invalid values (inf, nan, negative)
        valid_mask = np.isfinite(depth_np) & (depth_np > 0) & (depth_np < 10.0)
        valid_depths = depth_np[valid_mask]
        
        if len(valid_depths) > 0:
            stats = {
                'frame': frame_number,
                'min_depth': float(np.min(valid_depths)),
                'max_depth': float(np.max(valid_depths)),
                'mean_depth': float(np.mean(valid_depths)),
                'median_depth': float(np.median(valid_depths)),
                'std_depth': float(np.std(valid_depths)),
                'valid_pixels': int(np.sum(valid_mask)),
                'total_pixels': int(depth_np.size),
                'valid_ratio': float(np.sum(valid_mask) / depth_np.size)
            }
        else:
            stats = {
                'frame': frame_number,
                'min_depth': 0.0,
                'max_depth': 0.0,
                'mean_depth': 0.0,
                'median_depth': 0.0,
                'std_depth': 0.0,
                'valid_pixels': 0,
                'total_pixels': int(depth_np.size),
                'valid_ratio': 0.0
            }
        
        self.depth_stats.append(stats)
    
    def process_frame_advanced(self, depth_mat, point_cloud_mat, image_mat, frame_number):
        """
        Advanced frame processing combining StereoLabs and ROS2 depth analysis patterns
        """
        # Convert ZED Mat to numpy array (millimeter units from StereoLabs pattern)
        depth_np = depth_mat.get_data()  # This is in millimeters
        
        # Convert to meters for ROS2-style processing  
        depth_meters = depth_np / 1000.0
        
        # Get image dimensions for center point calculation (StereoLabs pattern)
        width = image_mat.get_width()
        height = image_mat.get_height()
        x_center = round(width / 2)
        y_center = round(height / 2)
        
        # Get distance at center using both methods (StereoLabs pattern)\n        
        # Method 1: Point cloud distance (3D Euclidean distance)
        err, point_cloud_value = point_cloud_mat.get_value(x_center, y_center)
        if err == sl.ERROR_CODE.SUCCESS:
            distance_3d = math.sqrt(\n                point_cloud_value[0] * point_cloud_value[0] +\n                point_cloud_value[1] * point_cloud_value[1] +\n                point_cloud_value[2] * point_cloud_value[2]\n            )\n            print(f\"Frame {frame_number} - 3D Distance to center ({x_center}, {y_center}): {distance_3d:.1f} mm\", end=\"\\r\")\n        \n        # Method 2: Direct depth value\n        err, depth_value = depth_mat.get_value(x_center, y_center)\n        if err == sl.ERROR_CODE.SUCCESS:\n            print(f\"Frame {frame_number} - Depth to center ({x_center}, {y_center}): {depth_value:.1f} mm\", end=\"\\r\")\n            self.center_distances.append(depth_value)\n        \n        # ROS2-style obstacle detection and analysis\n        valid_mask = np.logical_and(\n            depth_meters >= self.min_depth,\n            depth_meters <= self.max_depth\n        )\n        \n        # Find obstacles (objects closer than threshold) \n        obstacles = np.logical_and(\n            valid_mask,\n            depth_meters < self.obstacle_threshold\n        )\n        \n        # Calculate statistics\n        obstacle_pixel_count = np.sum(obstacles)\n        total_valid_pixels = np.sum(valid_mask)\n        \n        if total_valid_pixels > 0:\n            obstacle_percentage = (obstacle_pixel_count / total_valid_pixels) * 100\n            \n            # Find closest and average distances\n            valid_depth_values = depth_meters[valid_mask]\n            if len(valid_depth_values) > 0:\n                closest_distance = np.min(valid_depth_values)\n                average_distance = np.mean(valid_depth_values)\n                \n                # Store comprehensive statistics\n                stats = {\n                    'frame': frame_number,\n                    'min_depth': float(closest_distance),\n                    'max_depth': float(np.max(valid_depth_values)),\n                    'mean_depth': float(average_distance),\n                    'median_depth': float(np.median(valid_depth_values)),\n                    'std_depth': float(np.std(valid_depth_values)),\n                    'valid_pixels': int(total_valid_pixels),\n                    'obstacle_pixels': int(obstacle_pixel_count),\n                    'obstacle_percentage': float(obstacle_percentage),\n                    'center_depth_mm': float(depth_value) if 'depth_value' in locals() else 0.0,\n                    'center_3d_distance_mm': float(distance_3d) if 'distance_3d' in locals() else 0.0,\n                    'total_pixels': int(depth_meters.size),\n                    'valid_ratio': float(total_valid_pixels / depth_meters.size)\n                }\n                \n                # Real-time console output (ROS2 style)\n                print(f\"\\n📊 Frame {frame_number}: Closest={closest_distance:.2f}m | \"\n                      f\"Avg={average_distance:.2f}m | Obstacles={obstacle_percentage:.1f}% | \"\n                      f\"Center={depth_value/1000:.2f}m\")\n                \n                # Alert for close obstacles\n                if closest_distance < 0.5:  # Within 50cm\n                    print(\"⚠️  WARNING: Very close obstacle detected!\")\n                \n                self.depth_stats.append(stats)
    
    def save_depth_map(self, depth_mat, frame_number):
        """Save depth map as an image"""
        depth_np = depth_mat.get_data()
        
        # Normalize for visualization
        valid_mask = np.isfinite(depth_np) & (depth_np > 0)
        if np.any(valid_mask):
            depth_normalized = np.zeros_like(depth_np)
            valid_depths = depth_np[valid_mask]
            min_depth, max_depth = np.min(valid_depths), np.max(valid_depths)
            
            if max_depth > min_depth:
                depth_normalized[valid_mask] = ((depth_np[valid_mask] - min_depth) / 
                                               (max_depth - min_depth) * 255).astype(np.uint8)
        else:
            depth_normalized = np.zeros_like(depth_np, dtype=np.uint8)
        
        # Apply colormap
        depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
        
        # Save image
        filename = self.output_dir / "depth_maps" / f"depth_frame_{frame_number:06d}.png"
        cv2.imwrite(str(filename), depth_colored)
    
    def save_point_cloud(self, point_cloud_mat, frame_number):
        """Save point cloud as PLY file"""
        filename = self.output_dir / "point_clouds" / f"pointcloud_frame_{frame_number:06d}.ply"
        point_cloud_mat.write(str(filename))
    
    def generate_analysis_report(self):
        """Generate comprehensive analysis report"""
        if not self.depth_stats:
            print("No depth statistics to analyze")
            return
        
        # Save raw statistics as JSON
        stats_file = self.output_dir / "analysis" / "depth_statistics.json"
        with open(stats_file, 'w') as f:
            json.dump(self.depth_stats, f, indent=2)
        
        # Create plots
        self.create_depth_plots()
        
        # Generate summary report
        self.generate_summary_report()
    
    def create_depth_plots(self):
        """Create visualization plots for depth analysis"""
        frames = [s['frame'] for s in self.depth_stats]
        mean_depths = [s['mean_depth'] for s in self.depth_stats]
        valid_ratios = [s['valid_ratio'] for s in self.depth_stats]
        min_depths = [s['min_depth'] for s in self.depth_stats]
        max_depths = [s['max_depth'] for s in self.depth_stats]
        
        # Create subplots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Mean depth over time
        ax1.plot(frames, mean_depths)
        ax1.set_title('Mean Depth Over Time')
        ax1.set_xlabel('Frame Number')
        ax1.set_ylabel('Mean Depth (m)')
        ax1.grid(True)
        
        # Valid pixel ratio over time
        ax2.plot(frames, valid_ratios)
        ax2.set_title('Valid Pixel Ratio Over Time')
        ax2.set_xlabel('Frame Number')
        ax2.set_ylabel('Valid Pixel Ratio')
        ax2.grid(True)
        
        # Min/Max depth range
        ax3.fill_between(frames, min_depths, max_depths, alpha=0.3)
        ax3.plot(frames, min_depths, label='Min Depth')
        ax3.plot(frames, max_depths, label='Max Depth')
        ax3.set_title('Depth Range Over Time')
        ax3.set_xlabel('Frame Number')
        ax3.set_ylabel('Depth (m)')
        ax3.legend()
        ax3.grid(True)
        
        # Histogram of mean depths
        ax4.hist(mean_depths, bins=30, alpha=0.7)
        ax4.set_title('Distribution of Mean Depths')
        ax4.set_xlabel('Mean Depth (m)')
        ax4.set_ylabel('Frequency')
        ax4.grid(True)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / "analysis" / "depth_analysis.png", dpi=150)
        plt.close()
    
    def generate_summary_report(self):
        """Generate a summary report"""
        mean_depths = [s['mean_depth'] for s in self.depth_stats if s['mean_depth'] > 0]
        valid_ratios = [s['valid_ratio'] for s in self.depth_stats]
        
        summary = {
            'total_frames': len(self.depth_stats),
            'overall_mean_depth': float(np.mean(mean_depths)) if mean_depths else 0.0,
            'overall_std_depth': float(np.std(mean_depths)) if mean_depths else 0.0,
            'min_depth_recorded': float(min([s['min_depth'] for s in self.depth_stats if s['min_depth'] > 0], default=0)),
            'max_depth_recorded': float(max([s['max_depth'] for s in self.depth_stats])),
            'average_valid_ratio': float(np.mean(valid_ratios)),
            'frames_with_valid_depth': len([s for s in self.depth_stats if s['valid_pixels'] > 0])
        }
        
        # Save summary
        summary_file = self.output_dir / "analysis" / "summary_report.json"
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        # Print summary
        print("\n=== Analysis Summary ===")
        print(f"Total frames processed: {summary['total_frames']}")
        print(f"Frames with valid depth: {summary['frames_with_valid_depth']}")
        print(f"Overall mean depth: {summary['overall_mean_depth']:.3f}m")
        print(f"Average valid pixel ratio: {summary['average_valid_ratio']:.3f}")
        print(f"Depth range: {summary['min_depth_recorded']:.3f}m - {summary['max_depth_recorded']:.3f}m")
        print(f"Results saved to: {self.output_dir}")

def main():
    parser = argparse.ArgumentParser(description='ZED SDK Depth Analysis Tool')
    parser.add_argument('--input', '-i', required=True, help='Path to VSO file')
    parser.add_argument('--output', '-o', default='output', help='Output directory')
    parser.add_argument('--max-frames', '-f', type=int, help='Maximum number of frames to process')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input):
        print(f"Error: VSO file not found: {args.input}")
        return 1
    
    analyzer = DepthAnalyzerPython(args.output)
    success = analyzer.process_vso_file(args.input, args.max_frames)
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())