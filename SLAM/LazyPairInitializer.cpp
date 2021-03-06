#include <unordered_set>
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
    m_first_frame.reset();
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
        (real)config->value("Triangulation.sigma", 2.0*sigma)
    );

    m_min_parallax = (real)config->value("Triangulation.minParallax", 1.0f);
    m_K = config->K;
}

LazyPairInitializer::~LazyPairInitializer() = default;

void LazyPairInitializer::reset() {
    m_first_frame.reset();
}

bool LazyPairInitializer::initialize(const std::shared_ptr<Frame> &pframe) {
    if (m_first_frame) {
        m_frame_count++;
        match_vector matches = m_first_frame->feature->match(pframe->feature.get(), 5, 0.5f*m_K(0, 2) / m_K(0, 0));
        size_t N = matches.size();

        m_essential_ransac->set_dataset(m_first_frame->feature->keypoints, pframe->feature->keypoints, matches);
        m_essential_ransac->run();
        matches.swap(m_essential_ransac->matches);

        m_homography_ransac->set_dataset(m_first_frame->feature->keypoints, pframe->feature->keypoints, matches);
        m_homography_ransac->run();
        matches.swap(m_homography_ransac->matches);

        m_triangulator->set_dataset(m_first_frame->feature->keypoints, pframe->feature->keypoints, matches);
        m_triangulator->run(m_essential_ransac->E);
        matches.swap(m_triangulator->matches);

        real angle = acos(m_triangulator->parallax) * 180 / 3.1415927f;

        // test image area occupation
        // grid width and height
        real wx = m_K(0, 2) * 0.25f;
        real wy = m_K(1, 2) * 0.25f;
        std::unordered_set<int> grid;
        for (auto &m : matches) {
            auto &pt = pframe->feature->keypoints[m.second];
            int ix = (int)((pt[0] * m_K(0, 0)) / wx + 5);
            int iy = (int)((pt[1] * m_K(1, 1)) / wy + 5);
            grid.emplace(ix + iy * 100);
        }

        if (angle >= m_min_parallax && grid.size() >= 48) {

            OcvHelperFunctions::show_match_overlayed(m_first_frame->feature.get(), pframe->feature.get(), matches, 1);

            this->matches.swap(matches);
            this->points.swap(m_triangulator->points);

            m_first_frame->R = mat3::Identity();
            m_first_frame->T = vec3::Zero();

            pframe->R = m_triangulator->R;
            pframe->T = m_triangulator->T;

            reference_frame = m_first_frame;

            return true;
        }

        if (m_frame_count > 10 && matches.size() <= N / 10) {
            m_first_frame = pframe;
            m_frame_count = 0;
        }
    }
    else {
        m_first_frame = pframe;
        m_frame_count = 0;
    }

    return false;
}
