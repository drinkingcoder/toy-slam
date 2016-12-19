#pragma once

#include <memory>
#include <list>
#include "Types.h"

namespace slam {

    class Config;
    class Image;
    class Feature;
    class FeatureExtractor;
    class Initializer;

    class Frame {
    public:
        Frame(std::unique_ptr<Feature> &&feature);
        virtual ~Frame();

        std::unique_ptr<Feature> feature;
        bool is_keyframe = false;

        mat3 R;
        vec3 T;
    };

    class Tracker {
    public:
        Tracker(const Config *config);

        virtual ~Tracker();

        void track(const Image *image);

        Frame &current_frame() { return m_frames.back(); }
        const Frame &current_frame() const { return m_frames.back(); }

    private:
        enum TrackState { STATE_INITIALIZING, STATE_TRACKING, STATE_LOST } m_status;

        std::unique_ptr<FeatureExtractor> m_extractor;
        std::unique_ptr<Initializer> m_initializer;

        std::list<Frame> m_frames;
    };

}
