#include "Tracker.h"
#include "Config.h"
#include "Image.h"
#include "OcvOrbFeature.h"
#include "LazyPairInitializer.h"
#include "CeresMap.h"

#include "OcvHelperFunctions.h"

using namespace slam;

Frame::Frame(std::unique_ptr<Feature>&& feature) {
    this->feature = std::move(feature);
}

Frame::~Frame() = default;

Tracker::Tracker(const Config *config) {
    m_extractor = std::make_unique<OcvOrbFeatureExtractor>(config);
    m_initializer = std::make_unique<LazyPairInitializer>(config);
    m_map = std::make_unique<CeresMap>();
    m_status = STATE_INITIALIZING;
}

Tracker::~Tracker() = default;

void Tracker::track(const Image *image) {
    std::shared_ptr<Frame> pframe = std::make_shared<Frame>(m_extractor->extract(image));
    OcvHelperFunctions::show_image(1);

    if (m_status == STATE_INITIALIZING) {
        if (m_initializer->initialize(pframe)) {

            m_map->clear();

            auto &f1 = m_initializer->init_frame_1;
            auto &f2 = m_initializer->init_frame_2;

            size_t keyframe1 = m_map->add_keyframe(f1);
            size_t keyframe2 = m_map->add_keyframe(f2);

            for (size_t i = 0; i < m_initializer->points.size(); ++i) {
                size_t landmark = m_map->add_landmark(m_initializer->points[i]);
                m_map->add_observation(keyframe1, landmark, f1->feature->keypoints[m_initializer->matches[i].first]);
                m_map->add_observation(keyframe2, landmark, f2->feature->keypoints[m_initializer->matches[i].second]);
            }

            if (m_map->init(keyframe1, keyframe2)) {
                m_initializer->reset();
                m_status = STATE_TRACKING;
            }
        }
    }
    else {
        exit(0);
    }
}
