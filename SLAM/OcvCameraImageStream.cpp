#include "OcvCameraImageStream.h"
#include "OcvImage.h"
#include "OcvImage_Impl.h"
#include <opencv2/opencv.hpp>

namespace slam {

    struct OcvCameraImageStream_Impl {
        cv::Ptr<cv::VideoCapture> vc;
    };

}

using namespace slam;

OcvCameraImageStream::OcvCameraImageStream() {
    m_pimpl = std::make_unique<OcvCameraImageStream_Impl>();
    m_pimpl->vc.reset(new cv::VideoCapture(0));
    m_pimpl->vc->set(cv::CAP_PROP_SETTINGS, 1);
}

OcvCameraImageStream::~OcvCameraImageStream() = default;

std::unique_ptr<Image> OcvCameraImageStream::next() {
    auto img = std::make_unique<OcvImage>();
    (*(m_pimpl->vc)) >> (img->m_pimpl->image);
    if (!img->valid()) {
        return nullptr;
    }
    else {
        return img;
    }
}
