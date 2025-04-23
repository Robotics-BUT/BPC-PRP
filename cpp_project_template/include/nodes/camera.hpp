#pragma once

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

namespace algorithms {

    class ArucoDetector {
    public:
        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;
        };

        ArucoDetector();
        ~ArucoDetector() = default;

        // Detect ArUco markers from input OpenCV image
        std::vector<Aruco> detect(cv::Mat frame);

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
    };

}



