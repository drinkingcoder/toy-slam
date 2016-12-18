#include "OcvYamlConfig.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace slam;

namespace slam {

    struct OcvYamlConfig_Impl {
        cv::Ptr<cv::FileStorage> fs;
    };

}

OcvYamlConfig::OcvYamlConfig(const std::string & filepath) {
    m_pimpl = std::make_unique<OcvYamlConfig_Impl>();
    m_pimpl->fs.reset(new cv::FileStorage(filepath, cv::FileStorage::READ));
    if (!m_pimpl->fs->isOpened()) {
        std::cerr << "Cannot open config file: " << filepath << std::endl;
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
