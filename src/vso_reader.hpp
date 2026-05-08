#pragma once

#include <sl/Camera.hpp>
#include <string>

// Forward declaration
class DepthAnalyzer;

class VSOReader {
public:
    VSOReader(sl::Camera& camera);
    ~VSOReader();

    // Process the entire VSO file
    bool processFile(DepthAnalyzer& analyzer);
    
    // Get VSO file information
    void printFileInfo();
    
    // Skip to specific frame
    bool seekToFrame(int frame_number);

private:
    sl::Camera& camera_;
    int total_frames_;
    bool is_initialized_;
    
    // Helper methods
    void initializeReader();
};