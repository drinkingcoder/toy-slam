#pragma once

#include <memory>
#include "Types.h"

namespace slam {

    class Frame;

    class Initializer {
    public:
        virtual ~Initializer() {}

        virtual void reset() = 0;
        virtual bool initialize(const std::shared_ptr<Frame> &pframe) = 0;

        std::shared_ptr<Frame> reference_frame;
        match_vector matches;
        std::vector<vec3> points;
    };

}
