#pragma once

#include <memory>
#include "Types.h"

namespace slam {

    class Config;
    class Image;
    class Feature;
    class FeatureExtractor;
    class Initializer;
    class Map;

    class Frame {
    public:
        Frame(std::unique_ptr<Feature> &&feature);
        virtual ~Frame();

        std::unique_ptr<Feature> feature;

        mat3 R;
        vec3 T;
    };

    class Tracker {
    public:
        Tracker(const Config *config);

        virtual ~Tracker();

        void track(const Image *image);

    private:
        enum TrackState { STATE_INITIALIZING, STATE_TRACKING, STATE_LOST } m_status;

        std::unique_ptr<FeatureExtractor> m_extractor;
        std::unique_ptr<Initializer> m_initializer;
        std::unique_ptr<Map> m_map;
    };

}
