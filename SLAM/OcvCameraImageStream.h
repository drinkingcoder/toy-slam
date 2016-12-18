#pragma once

#include <memory>
#include "ImageStream.h"

namespace slam {

    struct OcvCameraImageStream_Impl;

    class OcvCameraImageStream : public ImageStream {
    public:
        OcvCameraImageStream();
        ~OcvCameraImageStream();

        virtual std::unique_ptr<Image> next();

    private:
        std::unique_ptr<OcvCameraImageStream_Impl> m_pimpl;
    };

}
