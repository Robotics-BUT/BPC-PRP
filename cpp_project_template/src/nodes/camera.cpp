#include "nodes/camera.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace algorithms {

    ArucoDetector::ArucoDetector() {
        // Používame 4x4 značky s 50 ID
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    }

    std::vector<ArucoDetector::Aruco> ArucoDetector::detect(cv::Mat frame) {
        std::vector<Aruco> arucos;
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        // Detekcia markerov
        cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

        if (!marker_ids.empty()) {
            std::cout << "Arucos found: ";
            for (size_t i = 0; i < marker_ids.size(); i++) {
                std::cout << marker_ids[i] << " ";
                arucos.push_back({marker_ids[i], marker_corners[i]});
            }
            std::cout << std::endl;
        }

        return arucos;
    }

}




