#pragma once

#include <memory>

namespace slam {

    class Frame;

    class Initializer {
    public:
        virtual ~Initializer() {}

        virtual bool initialize(const std::shared_ptr<Frame> &pframe) = 0;

    };

}
