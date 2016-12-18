#pragma once

#include <memory>
#include "Image.h"

namespace slam {

    class ImageStream {
    public:
        virtual ~ImageStream() {}

        virtual std::unique_ptr<Image> next() = 0;

    };

}
