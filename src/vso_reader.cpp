#include "vso_reader.hpp"
#include "depth_analyzer.hpp"
#include <iostream>
#include <iomanip>

VSOReader::VSOReader(sl::Camera& camera) 
    : camera_(camera), total_frames_(0), is_initialized_(false) {
    initializeReader();
}

VSOReader::~VSOReader() {
    // Cleanup if needed
}

void VSOReader::initializeReader() {
    if (!camera_.isOpened()) {
        std::cerr << "Camera is not opened!" << std::endl;
        return;
    }
    
    // Get VSO file information
    sl::CameraInformation camera_info = camera_.getCameraInformation();
    total_frames_ = camera_.getSVONumberOfFrames();
    
    std::cout << "VSO File Information:" << std::endl;
    std::cout << "Total frames: " << total_frames_ << std::endl;
    std::cout << "Resolution: " << camera_info.camera_configuration.resolution.width 
              << "x" << camera_info.camera_configuration.resolution.height << std::endl;
    std::cout << "FPS: " << camera_info.camera_configuration.fps << std::endl;
    
    is_initialized_ = true;
}

bool VSOReader::processFile(DepthAnalyzer& analyzer) {
    if (!is_initialized_) {
        std::cerr << "VSOReader not properly initialized!" << std::endl;
        return false;
    }
    
    std::cout << "Processing VSO file..." << std::endl;
    std::cout << "Progress: ";
    
    int frame_count = 0;
    int progress_step = std::max(1, total_frames_ / 20); // Update progress 20 times
    
    while (true) {
        sl::RuntimeParameters rt_params;
        sl::ERROR_CODE grab_status = camera_.grab(rt_params);
        
        if (grab_status == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            std::cout << "\nReached end of VSO file." << std::endl;
            break;
        }
        
        if (grab_status != sl::ERROR_CODE::SUCCESS) {
            std::cout << "\nError grabbing frame " << frame_count 
                      << ": " << grab_status << std::endl;
            continue;
        }
        
        // Process current frame
        if (!analyzer.processFrame(camera_, frame_count)) {
            std::cerr << "\nError processing frame " << frame_count << std::endl;
            return false;
        }
        
        frame_count++;
        
        // Update progress
        if (frame_count % progress_step == 0 || frame_count == total_frames_) {
            float progress = (float)frame_count / total_frames_ * 100.0f;
            std::cout << std::fixed << std::setprecision(1) << progress << "% ";
            std::cout.flush();
        }
    }
    
    std::cout << "\nProcessed " << frame_count << " frames successfully." << std::endl;
    return true;
}

void VSOReader::printFileInfo() {
    if (!is_initialized_) {
        std::cerr << "VSOReader not properly initialized!" << std::endl;
        return;
    }
    
    sl::CameraInformation camera_info = camera_.getCameraInformation();
    
    std::cout << "\n=== VSO File Detailed Information ===" << std::endl;
    std::cout << "Camera Model: " << camera_info.camera_model << std::endl;
    std::cout << "Serial Number: " << camera_info.serial_number << std::endl;
    std::cout << "Firmware: " << camera_info.camera_configuration.firmware_version << std::endl;
    std::cout << "Total Frames: " << total_frames_ << std::endl;
    std::cout << "Resolution: " << camera_info.camera_configuration.resolution.width 
              << " x " << camera_info.camera_configuration.resolution.height << std::endl;
    std::cout << "FPS: " << camera_info.camera_configuration.fps << std::endl;
    
    // Camera calibration info
    sl::CalibrationParameters calib = camera_info.camera_configuration.calibration_parameters;
    std::cout << "\nLeft Camera Calibration:" << std::endl;
    std::cout << "  fx: " << calib.left_cam.fx << ", fy: " << calib.left_cam.fy << std::endl;
    std::cout << "  cx: " << calib.left_cam.cx << ", cy: " << calib.left_cam.cy << std::endl;
    
    std::cout << "\nBaseline: " << calib.getCameraBaseline() << " mm" << std::endl;
    
    // Current VSO position
    int current_frame = camera_.getSVOPosition();
    std::cout << "Current Frame Position: " << current_frame << std::endl;
}

bool VSOReader::seekToFrame(int frame_number) {
    if (!is_initialized_) {
        std::cerr << "VSOReader not properly initialized!" << std::endl;
        return false;
    }
    
    if (frame_number < 0 || frame_number >= total_frames_) {
        std::cerr << "Frame number out of range: " << frame_number 
                  << " (valid range: 0-" << total_frames_ - 1 << ")" << std::endl;
        return false;
    }
    
    camera_.setSVOPosition(frame_number);
    
    int actual_position = camera_.getSVOPosition();
    if (actual_position == frame_number) {
        std::cout << "Successfully seeked to frame " << frame_number << std::endl;
        return true;
    } else {
        std::cerr << "Failed to seek to frame " << frame_number 
                  << ". Actual position: " << actual_position << std::endl;
        return false;
    }
}