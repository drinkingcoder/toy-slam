#pragma once

#include <memory>
#include <deque>

namespace slam {

    class Image;
    class Feature;
    class FeatureExtractor;
    class Initializer;

    class Frame {
    public:
        Frame(std::unique_ptr<Feature> &&feature);
        virtual ~Frame();

        std::unique_ptr<Feature> feature;
    };

    class Tracker {
    public:
        Tracker();

        virtual ~Tracker();

        void track(const Image *image);

        Frame &current_frame() { return m_frames.back(); }
        const Frame &current_frame() const { return m_frames.back(); }

    private:
        enum TrackState { STATE_INITIALIZING, STATE_TRACKING, STATE_LOST } m_status;

        std::unique_ptr<FeatureExtractor> m_extractor;
        std::unique_ptr<Initializer> m_initializer;

        std::deque<Frame> m_frames;
    };

}
