#pragma once

#include <string>
#include "ImageStream.h"

namespace slam {

    class OcvImageSequenceStream : public ImageStream {
    public:
        OcvImageSequenceStream(const std::string &pattern, int begin, int step = 1);
        ~OcvImageSequenceStream();

        std::unique_ptr<Image> next() override;

    private:
        int m_current;

        std::string m_pattern;
        int m_step;
    };

}
