#include "Tracker.h"
#include "Image.h"
#include "Feature.h"
#include "Initializer.h"

#include "OcvOrbFeature.h"

#include "OcvHelperFunctions.h"

using namespace slam;

Frame::Frame(std::unique_ptr<Feature>&& feature) {
    this->feature = std::move(feature);
}

Frame::~Frame() = default;

Tracker::Tracker() {
    m_extractor = std::make_unique<OcvOrbFeatureExtractor>();
    m_initializer = std::make_unique<LazyPairInitializer>();
    m_status = STATE_INITIALIZING;
}

Tracker::~Tracker() = default;

void Tracker::track(const Image *image) {
    slam::OcvHelperFunctions::show_image(image);

    m_frames.emplace_back(m_extractor->extract(image));

    if (m_status == STATE_INITIALIZING) {
        m_initializer->initialize(this);
    }
    else {
        exit(0);
    }
}
