#include "Tracker.h"
#include "Config.h"
#include "Image.h"
#include "Feature.h"
#include "LazyPairInitializer.h"

#include "OcvOrbFeature.h"

#include "OcvHelperFunctions.h"

using namespace slam;

Frame::Frame(std::unique_ptr<Feature>&& feature) {
    this->feature = std::move(feature);
}

Frame::~Frame() = default;

Tracker::Tracker(const Config *config) {
    m_extractor = std::make_unique<OcvOrbFeatureExtractor>(config);
    m_initializer = std::make_unique<LazyPairInitializer>(config);
    m_status = STATE_INITIALIZING;
}

Tracker::~Tracker() = default;

void Tracker::track(const Image *image) {
    m_frames.emplace_back(m_extractor->extract(image));
    if (m_status == STATE_INITIALIZING) {
        OcvHelperFunctions::current_image = image;
        m_initializer->initialize(this);
        OcvHelperFunctions::current_image = nullptr;
    }
}
