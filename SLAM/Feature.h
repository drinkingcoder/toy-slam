#pragma once

#include <vector>
#include <memory>
#include "Types.h"

namespace slam {

    class Image;

    class Feature {
    public:
        virtual ~Feature() {}

        virtual match_vector match(const Feature *) const = 0;

        std::vector<vec2> keypoints;
        
    };

    class FeatureExtractor {
    public:
        virtual ~FeatureExtractor() {}

        virtual std::unique_ptr<Feature> extract(const Image *image) const = 0;

    };

}
