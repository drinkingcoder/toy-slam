#pragma once

#include <memory>
#include "Types.h"

namespace slam {

    class Frame;
    class Initializer;

    class Map {
    public:
        virtual ~Map() {}

        virtual void clear() = 0;

        virtual size_t add_keyframe(const std::shared_ptr<Frame> &pframe) = 0;
        virtual size_t add_landmark(const vec3 &point) = 0;

        virtual void add_observation(size_t keyframe, size_t landmark, const vec2 &x) = 0;

        virtual bool init(const std::shared_ptr<Frame> &current_frame, const Initializer *initializer) = 0;

        virtual bool localize(const std::shared_ptr<Frame> &pframe) = 0;

    };

}
