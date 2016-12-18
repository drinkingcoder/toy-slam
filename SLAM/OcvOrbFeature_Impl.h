#pragma once

#include <opencv2/opencv.hpp>

namespace slam {

    struct OcvOrbFeature_Impl {
        cv::Mat descriptors;
    };

    struct OcvOrbFeatureExtractor_Impl {
        cv::Ptr<cv::Feature2D> fast;
        cv::Ptr<cv::Feature2D> orb;
    };

}
