#pragma once

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class DepthAnalyzer {
public:
    DepthAnalyzer(const std::string& output_directory);
    ~DepthAnalyzer();

    // Process a single frame and extract depth information
    bool processFrame(sl::Camera& zed, int frame_number);
    
    // Save depth map as image
    bool saveDepthMap(const sl::Mat& depth, int frame_number);
    
    // Save point cloud
    bool savePointCloud(sl::Camera& zed, int frame_number);
    
    // Analyze depth statistics
    void analyzeDepthStatistics(const sl::Mat& depth, int frame_number);
    
    // Generate analysis report
    void generateReport();

private:
    std::string output_dir_;
    std::vector<float> min_depths_;
    std::vector<float> max_depths_;
    std::vector<float> mean_depths_;
    std::vector<int> valid_pixels_;
    
    // Helper methods
    cv::Mat slMat2cvMat(const sl::Mat& input);
    bool createOutputDirectory();
};