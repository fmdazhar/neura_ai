#pragma once

#include <opencv2/opencv.hpp>

namespace edge_detection {

class EdgeDetector {
public:
    int cannyLowThreshold_;
    int cannyHighThreshold_;

    EdgeDetector(int cannyLowThreshold = 50, int cannyHighThreshold = 150)
        : cannyLowThreshold_(cannyLowThreshold),
          cannyHighThreshold_(cannyHighThreshold) {}

    cv::Mat detectEdges(const cv::Mat& bgrImage) {
        cv::Mat gray, blurred, edges;
        cv::cvtColor(bgrImage, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        cv::Canny(blurred, edges, cannyLowThreshold_, cannyHighThreshold_);
        return edges;
    }
};

} // namespace edge_detection
