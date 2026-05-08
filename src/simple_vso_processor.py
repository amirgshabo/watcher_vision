#!/usr/bin/env python3
"""
Simple VSO File Processor - StereoLabs Pattern
==============================================

This script uses the exact StereoLabs code pattern you provided
for quick testing and validation of VSO files.
"""

import pyzed.sl as sl
import math
import cv2
import numpy as np

def process_vso_simple(vso_path, max_frames=50, start_frame=500):
    """
    Process VSO file using exact StereoLabs pattern
    """
    print(f"Processing VSO file: {vso_path}")
    print("Using StereoLabs reference pattern")
    print("=" * 50)
    
    # Create a ZED camera (exact StereoLabs pattern)
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(vso_path)  # Load from VSO file
    init_params.sdk_verbose = 1  # Enable verbose logging (1 = True)
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Use NEURAL mode for better depth quality
    init_params.coordinate_units = sl.UNIT.METER  # Use meters for easier reading

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Error {}, exit program".format(err)) # Display the error
        return False

    # Get camera info
    camera_info = zed.get_camera_information()
    total_frames = zed.get_svo_number_of_frames()
    resolution = camera_info.camera_configuration.resolution
    print(f"VSO Info: {total_frames} frames, Resolution: {resolution.width}x{resolution.height}")
    print(f"Starting from frame {start_frame}, processing {max_frames} frames...")
    print()

    # Skip to start frame
    if start_frame > 0 and start_frame < total_frames:
        zed.set_svo_position(start_frame)
        print(f"Skipped to frame {start_frame}")

    # Capture frames and depth (StereoLabs pattern)
    i = 0
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    
    distances_center = []
    
    while i < max_frames and i < total_frames:
        # Grab an image
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns sl.ERROR_CODE.SUCCESS
            zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH) # Retrieve depth matrix. Depth is aligned on the left RGB image
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA) # Retrieve colored point cloud
            
            # Get the camera image for display
            image_np = image.get_data()
            image_bgr = cv2.cvtColor(image_np, cv2.COLOR_RGBA2BGR)
            
            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            x = round(image.get_width() / 2)
            y = round(image.get_height() / 2)
            
            # Method 1: Point cloud distance
            err, point_cloud_value = point_cloud.get_value(x, y)
            if err == sl.ERROR_CODE.SUCCESS:
                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                   point_cloud_value[1] * point_cloud_value[1] +
                                   point_cloud_value[2] * point_cloud_value[2])
                # Only print every 10 frames to reduce spam
                if i % 10 == 0:
                    print("Frame {0}: 3D Distance = {1:.1f}mm".format(i+1, distance))
            
            # Method 2: Direct depth value
            err, depth_value = depth.get_value(x, y)
            if err == sl.ERROR_CODE.SUCCESS and not np.isnan(depth_value) and depth_value > 0:
                # Only print every 10 frames
                if i % 10 == 0:
                    print("Frame {0}: Depth = {1:.2f}m".format(i+1, depth_value))
                distances_center.append(depth_value)
            else:
                # Try to find a valid depth point nearby
                depth_np = depth.get_data()
                if depth_np is not None:
                    # Look for valid depths in a small region around center
                    center_region = depth_np[y-50:y+50, x-50:x+50]
                    valid_depths = center_region[np.isfinite(center_region) & (center_region > 0)]
                    if len(valid_depths) > 0:
                        avg_depth = np.mean(valid_depths)
                        if i % 10 == 0:
                            print("Frame {0}: Avg region = {1:.2f}m".format(i+1, avg_depth))
                        distances_center.append(avg_depth)
                    else:
                        if i % 10 == 0:
                            print("Frame {0}: No valid depth".format(i+1))
            # ALWAYS show visualization for EVERY frame  
            # Get depth data for visualization (ALWAYS do this)
            depth_np = depth.get_data()
            
            if depth_np is not None and 'image_bgr' in locals():
                # Resize images for side-by-side display
                height, width = image_bgr.shape[:2]
                display_width = 640
                display_height = int(height * display_width / width)
                
                # Resize camera image
                camera_resized = cv2.resize(image_bgr, (display_width, display_height))
                
                # Process depth for display (FIXED: closer = red, farther = blue)
                valid_mask = np.isfinite(depth_np) & (depth_np > 0)
                if np.any(valid_mask):
                    depth_for_display = depth_np.copy()
                    depth_for_display[~valid_mask] = np.nan
                    
                    # Get min/max for proper scaling
                    min_depth = np.nanmin(depth_for_display)
                    max_depth = np.nanmax(depth_for_display)
                    
                    # Invert the mapping: closer objects (smaller depth) = higher values = red
                    # Farther objects (larger depth) = lower values = blue
                    depth_for_display[~valid_mask] = max_depth  # Set invalid to max (blue)
                    depth_inverted = max_depth - depth_for_display + min_depth
                    
                    # Normalize inverted depth to 0-255
                    depth_display = cv2.normalize(depth_inverted, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                else:
                    depth_display = np.zeros_like(depth_np, dtype=np.uint8)
                
                depth_colored = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
                depth_resized = cv2.resize(depth_colored, (display_width, display_height))
                
                # Add center point to both images
                center_x = int(x * display_width / width)
                center_y = int(y * display_height / height)
                
                cv2.circle(camera_resized, (center_x, center_y), 8, (0, 255, 0), 2)
                cv2.circle(depth_resized, (center_x, center_y), 8, (255, 255, 255), 2)
                
                # Add depth value overlay
                if err == sl.ERROR_CODE.SUCCESS and not np.isnan(depth_value) and depth_value > 0:
                    depth_text = f"{depth_value:.1f}m"
                else:
                    depth_text = "NO_DEPTH"
                
                cv2.putText(camera_resized, depth_text, (center_x+10, center_y-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(depth_resized, depth_text, (center_x+10, center_y-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Create side-by-side display
                combined = np.hstack([camera_resized, depth_resized])
                
                # Add labels
                cv2.putText(combined, "CAMERA", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(combined, "DEPTH", (display_width + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                
                # Add frame info
                progress_pct = ((i+1)/max_frames)*100
                cv2.putText(combined, f"Frame {i+1}/{max_frames} ({progress_pct:.1f}%)", (10, display_height - 10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Show combined view with proper refresh
                cv2.imshow('ZED Live Processing - Camera & Depth', combined)
                cv2.waitKey(1)  # Force immediate display
                
                # Check for quit key
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\\n⏹️  User requested stop")
                    break
                elif key == ord(' '):  # Spacebar to pause
                    print("\\n⏸️  Paused - press any key to continue")
                    cv2.waitKey(0)
            
            else:
                print(f"Frame {i+1}: Failed to grab frame")
            
            # Update frame counter
            i = i + 1
    
    # Close camera
    zed.close()
    cv2.destroyAllWindows()
    
    # Summary
    if distances_center:
        valid_distances = [d for d in distances_center if not np.isnan(d)]
        if valid_distances:
            print(f"\\nSUMMARY:")
            print(f"Frames processed: {len(distances_center)}")
            print(f"Valid depth measurements: {len(valid_distances)}")
            print(f"Average center distance: {np.mean(valid_distances):.2f}m")
            print(f"Min center distance: {np.min(valid_distances):.2f}m") 
            print(f"Max center distance: {np.max(valid_distances):.2f}m")
        else:
            print(f"\\nSUMMARY:")
            print(f"Frames processed: {len(distances_center)}")
            print(f"No valid depth data found - this might be sky/distant scene")
    else:
        print(f"\\nNo depth measurements recorded")
    
    return True

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Simple VSO File Processor - StereoLabs Pattern')
    parser.add_argument('--input', '-i', required=True, help='Path to VSO file')
    parser.add_argument('--frames', '-f', type=int, default=10, help='Number of frames to process')
    parser.add_argument('--start', '-s', type=int, default=500, help='Frame number to start from')
    
    args = parser.parse_args()
    
    print("Simple VSO Processor")
    print("Using exact StereoLabs reference pattern")
    print("=" * 50)
    
    if not os.path.exists(args.input):
        print(f"Error: VSO file not found: {args.input}")
        return 1
    
    success = process_vso_simple(args.input, args.frames, args.start)
    return 0 if success else 1

if __name__ == "__main__":
    import os
    exit(main())