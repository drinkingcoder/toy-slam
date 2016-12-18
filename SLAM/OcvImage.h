#pragma once

#include <string>
#include <memory>

#include "Image.h"

namespace slam {

    struct OcvImage_Impl;

    class OcvImage : public Image {
        friend struct OcvHelperFunctions;
    public:
        OcvImage();
        OcvImage(const std::string &filepath);
        ~OcvImage();

        bool valid() const override;

    private:
        friend class OcvOrbFeatureExtractor;
        std::unique_ptr<OcvImage_Impl> m_pimpl;
    };

}
