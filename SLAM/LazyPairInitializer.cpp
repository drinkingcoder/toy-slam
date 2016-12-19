#include "LazyPairInitializer.h"
#include "Config.h"
#include "Tracker.h"
#include "Feature.h"
#include "EightPointEssentialRANSAC.h"
#include "FourPointHomographyRANSAC.h"
#include "Triangulator.h"
#include "OcvHelperFunctions.h"

using namespace slam;

LazyPairInitializer::LazyPairInitializer(const Config *config) {
    m_first_frame = nullptr;
    m_frame_count = 0;

    real sigma = (real)config->value("RANSAC.sigma", 1.0);
    real success_rate = (real)config->value("RANSAC.successRate", 0.99);
    size_t max_iter = (size_t)config->value("RANSAC.maxIteration", 10000000);

    m_essential_ransac = std::make_unique<EightPointEssentialRANSAC>(
        config->K,
        (real)config->value("RANSAC.Essential.sigma", sigma),
        (real)config->value("RANSAC.Essential.successRate", success_rate),
        (size_t)config->value("RANSAC.Essential.maxIteration", (double)max_iter)
    );

    m_homography_ransac = std::make_unique<FourPointHomographyRANSAC>(
        config->K,
        (real)config->value("RANSAC.Essential.sigma", sigma*10),
        (real)config->value("RANSAC.Essential.successRate", success_rate),
        (size_t)config->value("RANSAC.Essential.maxIteration", (double)max_iter)
    );

    m_triangulator = std::make_unique<Triangulator>(
        config->K,
        (real)config->value("Triangulation.sigma", sigma),
        (real)config->value("Triangulation.minParallax", 0.0),
        (size_t)config->value("Triangulation.minTriangulated", 0)
    );
}

LazyPairInitializer::~LazyPairInitializer() = default;

void LazyPairInitializer::initialize(Tracker *tracker) {
    Frame &current_frame = tracker->current_frame();
    if (m_first_frame) {
        m_frame_count++;
        match_vector matches = current_frame.feature->match(m_first_frame->feature.get());

        m_essential_ransac->set_dataset(current_frame.feature->keypoints, m_first_frame->feature->keypoints, matches);
        m_essential_ransac->run();
        matches.swap(m_essential_ransac->matches);

        m_homography_ransac->set_dataset(current_frame.feature->keypoints, m_first_frame->feature->keypoints, matches);
        m_homography_ransac->run();
        matches.swap(m_homography_ransac->matches);

        m_triangulator->set_dataset(current_frame.feature->keypoints, m_first_frame->feature->keypoints, matches);
        m_triangulator->run(m_essential_ransac->E);
        matches.swap(m_triangulator->matches);

        OcvHelperFunctions::show_match_overlayed(current_frame.feature.get(), m_first_frame->feature.get(), matches, 1);

        if (m_frame_count > 10 && matches.size() == 0) {
            m_first_frame = &current_frame;
            m_frame_count = 0;
        }
    }
    else {
        m_first_frame = &current_frame;
        m_frame_count = 0;
    }
}
