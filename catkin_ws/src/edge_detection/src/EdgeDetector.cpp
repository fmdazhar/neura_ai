//EdgeDetector.cpp
#include "edge_detection/EdgeDetector.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <sys/stat.h>

bool createDirectory(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        if (mkdir(path.c_str(), 0755) != 0) {
            std::cerr << "Error creating directory: " << path << "\n";
            return false;
        }
    } else if (!(info.st_mode & S_IFDIR)) {
        std::cerr << "Path exists but is not a directory: " << path << "\n";
        return false;
    }
    return true;
}

std::string getBaseName(const std::string& filepath) {
    size_t lastSlash = filepath.find_last_of("/\\");
    size_t start = (lastSlash == std::string::npos) ? 0 : lastSlash + 1;
    size_t dot = filepath.find_last_of('.');
    if (dot == std::string::npos || dot < start)
        return filepath.substr(start);
    return filepath.substr(start, dot - start);
}

int main() {
    const std::string inputFolder = "/root/catkin_ws/src/edge_detection/data/images";
    const std::string outputFolder = "/root/catkin_ws/results/basic/cpp";

    if (!createDirectory(outputFolder))
        return -1;

    std::vector<std::string> extensions = {"*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tiff"};
    std::vector<cv::String> imageFiles;

    for (const auto& ext : extensions) {
        std::vector<cv::String> files;
        cv::glob(inputFolder + "/" + ext, files, false);
        imageFiles.insert(imageFiles.end(), files.begin(), files.end());
    }

    std::cout << "Found " << imageFiles.size() << " images.\n";
    if (imageFiles.empty()) {
        std::cerr << "No images found in: " << inputFolder << "\n";
        return 0;
    }

    edge_detection::EdgeDetector detector(50, 150);

    for (const auto& imgPath : imageFiles) {
        cv::Mat img = cv::imread(imgPath);
        if (img.empty()) {
            std::cerr << "Failed to read " << imgPath << "\n";
            continue;
        }

        cv::Mat edges = detector.detectEdges(img);

        std::ostringstream oss;
        oss << outputFolder << "/" << getBaseName(imgPath) << "_canny.png";

        if (!cv::imwrite(oss.str(), edges))
            std::cerr << "Failed to write " << oss.str() << "\n";
        else
            std::cout << "Processed: " << getBaseName(imgPath) << "\n";
    }

    return 0;
}
