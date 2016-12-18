#include "Tracker.h"
#include "Config.h"
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

Tracker::Tracker(const Config *config) {
    m_extractor = std::make_unique<OcvOrbFeatureExtractor>(config);
    m_initializer = std::make_unique<LazyPairInitializer>();
    m_status = STATE_INITIALIZING;
}

Tracker::~Tracker() = default;

void Tracker::track(const Image *image) {
    //m_frames.emplace_back(m_extractor->extract(image));

    //auto feature = m_extractor->extract(image);

    //slam::OcvHelperFunctions::show_keypoints(image, feature.get(), 1);

    //if (m_status == STATE_INITIALIZING) {
    //    m_initializer->initialize(this);
    //}
    //else {
    //    exit(0);
    //}

    //while (m_frames.size() > 10) { m_frames.pop_front(); }
}
