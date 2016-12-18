#include "OcvImage.h"
#include "OcvImage_Impl.h"

using namespace slam;

OcvImage::OcvImage() {
    m_pimpl = std::make_unique<OcvImage_Impl>();
}

OcvImage::OcvImage(const std::string &filepath) {
    m_pimpl = std::make_unique<OcvImage_Impl>();
    m_pimpl->image = cv::imread(filepath);
    if (m_pimpl->image.channels() == 3) {
        cv::cvtColor(m_pimpl->image, m_pimpl->image, cv::COLOR_RGB2GRAY);
    }
}

OcvImage::~OcvImage() = default;

bool OcvImage::valid() const {
    return !(m_pimpl->image.empty());
}
