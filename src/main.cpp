#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

#include "depth_analyzer.hpp"
#include "vso_reader.hpp"

void printUsage() {
    std::cout << "Usage: depth_tester <path_to_vso_file> [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --output <dir>    Output directory for results (default: output/)\n";
    std::cout << "  --resolution <res> Resolution: HD720, HD1080, HD2K (default: HD720)\n";
    std::cout << "  --depth-mode <mode> Depth mode: NONE, PERFORMANCE, QUALITY, ULTRA (default: PERFORMANCE)\n";
    std::cout << "  --help            Show this help message\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return -1;
    }

    std::string vso_path = argv[1];
    std::string output_dir = "output/";
    sl::RESOLUTION resolution = sl::RESOLUTION::HD720;
    sl::DEPTH_MODE depth_mode = sl::DEPTH_MODE::PERFORMANCE;

    // Parse command line arguments
    for (int i = 2; i < argc; i++) {
        if (std::string(argv[i]) == "--help") {
            printUsage();
            return 0;
        } else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (std::string(argv[i]) == "--resolution" && i + 1 < argc) {
            std::string res = argv[++i];
            if (res == "HD720") resolution = sl::RESOLUTION::HD720;
            else if (res == "HD1080") resolution = sl::RESOLUTION::HD1080;
            else if (res == "HD2K") resolution = sl::RESOLUTION::HD2K;
        } else if (std::string(argv[i]) == "--depth-mode" && i + 1 < argc) {
            std::string mode = argv[++i];
            if (mode == "NONE") depth_mode = sl::DEPTH_MODE::NONE;
            else if (mode == "PERFORMANCE") depth_mode = sl::DEPTH_MODE::PERFORMANCE;
            else if (mode == "QUALITY") depth_mode = sl::DEPTH_MODE::QUALITY;
            else if (mode == "ULTRA") depth_mode = sl::DEPTH_MODE::ULTRA;
        }
    }

    std::cout << "ZED SDK Depth Testing Application\n";
    std::cout << "VSO File: " << vso_path << "\n";
    std::cout << "Output Directory: " << output_dir << "\n";

    try {
        // Initialize ZED Camera with VSO file (StereoLabs pattern)
        sl::Camera zed;
        sl::InitParameters init_params;
        init_params.input.setFromSVOFile(vso_path.c_str());
        init_params.sdk_verbose = true;  // Enable verbose logging (StereoLabs pattern)
        init_params.camera_resolution = resolution;
        init_params.depth_mode = depth_mode; // Set to performance (fastest)
        init_params.coordinate_units = sl::UNIT::MILLIMETER;  // Use millimeter units (StereoLabs default)

        sl::ERROR_CODE err = zed.open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            std::cerr << "Error opening ZED camera: " << err << std::endl;
            return -1;
        }

        // Initialize depth analyzer
        DepthAnalyzer analyzer(output_dir);
        
        // Process VSO file
        VSOReader reader(zed);
        if (!reader.processFile(analyzer)) {
            std::cerr << "Error processing VSO file" << std::endl;
            return -1;
        }

        std::cout << "Processing completed successfully!" << std::endl;
        
        // Close camera
        zed.close();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}