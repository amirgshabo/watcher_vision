#include "depth_analyzer.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>

DepthAnalyzer::DepthAnalyzer(const std::string& output_directory) 
    : output_dir_(output_directory) {
    createOutputDirectory();
}

DepthAnalyzer::~DepthAnalyzer() {
    generateReport();
}

bool DepthAnalyzer::createOutputDirectory() {
    try {
        std::filesystem::create_directories(output_dir_);
        std::filesystem::create_directories(output_dir_ + "/depth_maps");
        std::filesystem::create_directories(output_dir_ + "/point_clouds");
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error creating output directories: " << e.what() << std::endl;
        return false;
    }
}

bool DepthAnalyzer::processFrame(sl::Camera& zed, int frame_number) {
    sl::Mat image, depth, point_cloud;
    sl::RuntimeParameters rt_params;
    
    if (zed.grab(rt_params) == sl::ERROR_CODE::SUCCESS) {
        // Retrieve the left image and depth map
        zed.retrieveImage(image, sl::VIEW::LEFT);
        zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);
        
        // Save depth map
        if (!saveDepthMap(depth, frame_number)) {
            std::cerr << "Failed to save depth map for frame " << frame_number << std::endl;
            return false;
        }
        
        // Analyze depth statistics
        analyzeDepthStatistics(depth, frame_number);
        
        // Save point cloud (every 10th frame to save space)
        if (frame_number % 10 == 0) {
            if (!savePointCloud(zed, frame_number)) {
                std::cerr << "Failed to save point cloud for frame " << frame_number << std::endl;
            }
        }
        
        return true;
    }
    
    return false;
}

bool DepthAnalyzer::saveDepthMap(const sl::Mat& depth, int frame_number) {
    cv::Mat depth_cv = slMat2cvMat(depth);
    
    // Normalize depth for visualization
    cv::Mat depth_normalized;
    cv::normalize(depth_cv, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    
    // Apply colormap
    cv::Mat depth_colored;
    cv::applyColorMap(depth_normalized, depth_colored, cv::COLORMAP_JET);
    
    std::string filename = output_dir_ + "/depth_maps/depth_frame_" + 
                          std::to_string(frame_number) + ".png";
    
    return cv::imwrite(filename, depth_colored);
}

bool DepthAnalyzer::savePointCloud(sl::Camera& zed, int frame_number) {
    sl::Mat point_cloud;
    zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZ);
    
    std::string filename = output_dir_ + "/point_clouds/pointcloud_frame_" + 
                          std::to_string(frame_number) + ".ply";
    
    sl::ERROR_CODE err = point_cloud.write(filename.c_str());
    return (err == sl::ERROR_CODE::SUCCESS);
}

void DepthAnalyzer::analyzeDepthStatistics(const sl::Mat& depth, int frame_number) {
    cv::Mat depth_cv = slMat2cvMat(depth);
    
    // Calculate statistics
    cv::Scalar mean, stddev;
    cv::meanStdDev(depth_cv, mean, stddev);
    
    double min_val, max_val;
    cv::minMaxLoc(depth_cv, &min_val, &max_val);
    
    // Count valid pixels (non-zero, finite values)
    cv::Mat mask = (depth_cv > 0) & (depth_cv < 10.0); // Valid depth range 0-10m
    int valid_count = cv::countNonZero(mask);
    
    // Store statistics
    min_depths_.push_back(static_cast<float>(min_val));
    max_depths_.push_back(static_cast<float>(max_val));
    mean_depths_.push_back(static_cast<float>(mean[0]));
    valid_pixels_.push_back(valid_count);
    
    // Print frame statistics
    std::cout << "Frame " << frame_number 
              << " - Min: " << std::fixed << std::setprecision(3) << min_val << "m"
              << ", Max: " << max_val << "m"
              << ", Mean: " << mean[0] << "m"
              << ", Valid pixels: " << valid_count << std::endl;
}

void DepthAnalyzer::generateReport() {
    if (min_depths_.empty()) return;
    
    std::string report_file = output_dir_ + "/analysis_report.csv";
    std::ofstream file(report_file);
    
    if (!file.is_open()) {
        std::cerr << "Failed to create analysis report" << std::endl;
        return;
    }
    
    // Write CSV header
    file << "Frame,Min_Depth(m),Max_Depth(m),Mean_Depth(m),Valid_Pixels\n";
    
    // Write data
    for (size_t i = 0; i < min_depths_.size(); ++i) {
        file << i << "," 
             << std::fixed << std::setprecision(3) 
             << min_depths_[i] << ","
             << max_depths_[i] << ","
             << mean_depths_[i] << ","
             << valid_pixels_[i] << "\n";
    }
    
    file.close();
    
    // Print summary
    std::cout << "\nAnalysis Summary:\n";
    std::cout << "Total frames processed: " << min_depths_.size() << "\n";
    std::cout << "Analysis report saved to: " << report_file << "\n";
}

cv::Mat DepthAnalyzer::slMat2cvMat(const sl::Mat& input) {
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>());
}