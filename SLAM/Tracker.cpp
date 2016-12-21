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
    m_map = std::make_unique<CeresMap>(config);
    m_status = STATE_INITIALIZING;
}

Tracker::~Tracker() = default;

void Tracker::track(const Image *image) {
    std::shared_ptr<Frame> pframe = std::make_shared<Frame>(m_extractor->extract(image));
    OcvHelperFunctions::current_image = image;
    OcvHelperFunctions::show_keypoints(pframe->feature.get(), 1);
    if (m_status == STATE_INITIALIZING) {
        if (m_initializer->initialize(pframe)) {
            if (m_map->init(pframe, m_initializer.get())) {
                m_initializer->reset();
                m_status = STATE_TRACKING;
            }
        }
    }
    else if (m_status == STATE_TRACKING) {
        if (!m_map->localize(pframe)) {
            m_status = STATE_LOST;
        }
    }
    else if (m_status == STATE_LOST) {
        m_map->clear();
        m_status = STATE_INITIALIZING;
    }
}
