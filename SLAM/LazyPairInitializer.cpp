#include "LazyPairInitializer.h"
#include "Config.h"
#include "Tracker.h"
#include "Feature.h"
#include "RANSAC.h"
#include "OcvHelperFunctions.h"

using namespace slam;

LazyPairInitializer::LazyPairInitializer(const Config *config) {
    m_first_frame = nullptr;

    real sigma = (real)config->value("RANSAC.sigma", 1.0);
    real success_rate = (real)config->value("RANSAC.successRate", 0.99);
    size_t max_iter = (size_t)config->value("RANSAC.maxIteration", 10000000);

    m_essential_ransac = std::make_unique<EightPointEssentialRANSAC>(
        config->K,
        (real)config->value("RANSAC.Essential.sigma", sigma),
        (real)config->value("RANSAC.Essential.successRate", success_rate),
        (size_t)config->value("RANSAC.Essential.maxIteration", (double)max_iter)
    );
}

LazyPairInitializer::~LazyPairInitializer() = default;

void LazyPairInitializer::initialize(Tracker *tracker) {
    Frame &current_frame = tracker->current_frame();
    if (m_first_frame) {
        match_vector matches = current_frame.feature->match(m_first_frame->feature.get());
        m_essential_ransac->set_dataset(current_frame.feature->keypoints, m_first_frame->feature->keypoints, matches);
        m_essential_ransac->run();
        std::cout << m_essential_ransac->iter << ", " << m_essential_ransac->matches.size() << std::endl;
        OcvHelperFunctions::show_match_overlayed(current_frame.feature.get(), m_first_frame->feature.get(), m_essential_ransac->matches);
    }
    else {
        m_first_frame = &current_frame;
    }
}
