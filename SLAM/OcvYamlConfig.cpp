#include "OcvYamlConfig.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "OcvHelperFunctions.h"

using namespace slam;

namespace slam {

    class Image;

    struct OcvYamlConfig_Impl {
        cv::Ptr<cv::FileStorage> fs;
    };

}

mat3 OcvHelperFunctions::K = mat3::Identity();
const Image *OcvHelperFunctions::current_image = nullptr;

OcvYamlConfig::OcvYamlConfig(const std::string & filepath) {
    m_pimpl = std::make_unique<OcvYamlConfig_Impl>();
    m_pimpl->fs.reset(new cv::FileStorage(filepath, cv::FileStorage::READ));
    if (!m_pimpl->fs->isOpened()) {
        std::cerr << "Cannot open config file: " << filepath << std::endl;
    }
    else {
        (*(m_pimpl->fs))["Calib.fx"] >> K(0, 0);
        (*(m_pimpl->fs))["Calib.fy"] >> K(1, 1);
        (*(m_pimpl->fs))["Calib.cx"] >> K(0, 2);
        (*(m_pimpl->fs))["Calib.cy"] >> K(1, 2);
        OcvHelperFunctions::K = K;
    }
}

OcvYamlConfig::~OcvYamlConfig() = default;

std::string OcvYamlConfig::text(const std::string & config, const std::string & def) const {
    const cv::FileNode &n = (*(m_pimpl->fs))[config];
    if (n.isNone()) {
        return def;
    }
    else {
        return (std::string)n;
    }
}

double OcvYamlConfig::value(const std::string & config, const double & def) const {
    const cv::FileNode &n = (*(m_pimpl->fs))[config];
    if (n.isNone()) {
        return def;
    }
    else {
        return (double)n;
    }
}
