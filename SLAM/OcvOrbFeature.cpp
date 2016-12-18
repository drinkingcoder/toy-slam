#include <unordered_map>
#include "Config.h"
#include "OcvOrbFeature.h"
#include "OcvOrbFeature_Impl.h"
#include "OcvImage.h"
#include "OcvImage_Impl.h"

using namespace slam;

OcvOrbFeature::OcvOrbFeature() {
    m_pimpl = std::make_unique<OcvOrbFeature_Impl>();
}

OcvOrbFeature::~OcvOrbFeature() = default;

match_vector OcvOrbFeature::match(const Feature *feature) const {
    const OcvOrbFeature *cvfeature = dynamic_cast<const OcvOrbFeature*>(feature);
    if (cvfeature == nullptr) {
        return match_vector();
    }

    if (keypoints.size() == 0 || feature->keypoints.size() == 0) {
        return match_vector();
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> cvresult;
    matcher.match(m_pimpl->descriptors, cvfeature->m_pimpl->descriptors, cvresult);

    match_vector result(cvresult.size());
    for (size_t i = 0; i < cvresult.size(); ++i) {
        result[i].first = cvresult[i].queryIdx;
        result[i].second = cvresult[i].trainIdx;
    }

    return result;
}

static void spread_keypoints(std::vector<cv::KeyPoint> &cvkeypoints, int grid_size = 10) {
    union Hasher {
        struct {
            std::int16_t x;
            std::int16_t y;
        } fields;
        std::int32_t hash;
    } hasher;
    std::unordered_map<std::int32_t, cv::KeyPoint> grid;
    for (auto &key : cvkeypoints) {
        hasher.fields.x = (std::int16_t)(key.pt.x / grid_size);
        hasher.fields.y = (std::int16_t)(key.pt.y / grid_size);
        int hash = hasher.hash;
        if (grid.count(hash)) {
            if (grid.at(hash).response < key.response) {
                grid.at(hash) = key;
            }
        }
        else {
            grid[hash] = key;
        }
    }
    cvkeypoints.resize(grid.size());
    size_t id = 0;
    for (auto &key : grid) {
        cvkeypoints[id] = key.second;
        id++;
    }
}

OcvOrbFeatureExtractor::OcvOrbFeatureExtractor(const Config *config) {
    m_pimpl = std::make_unique<OcvOrbFeatureExtractor_Impl>();
    m_pimpl->fast = cv::FastFeatureDetector::create((int)config->value("FAST.threshold", 10), true);
    m_pimpl->orb = cv::ORB::create(0,
        (float)config->value("ORB.scaleFactor", 1.2),
        (int)config->value("ORB.nlevels", 8),
        (int)config->value("ORB.edgeThreshold", 31)
    );
    m_K = config->K;
    m_spread_size = (int)config->value("FAST.spread", 20);
}

OcvOrbFeatureExtractor::~OcvOrbFeatureExtractor() = default;

std::unique_ptr<Feature> OcvOrbFeatureExtractor::extract(const Image *image) const {
    const OcvImage *cvimage = dynamic_cast<const OcvImage *>(image);
    if (cvimage == nullptr || !cvimage->valid()) {
        return nullptr; // cannot extract on other image type;
    }

    const cv::Mat &cvmat = cvimage->m_pimpl->image;
    std::vector<cv::KeyPoint> cvkeypoints;
    std::unique_ptr<OcvOrbFeature> result = std::make_unique<OcvOrbFeature>();

    m_pimpl->fast->detect(cvmat, cvkeypoints);

    spread_keypoints(cvkeypoints, m_spread_size);

    m_pimpl->orb->compute(cvmat, cvkeypoints, result->m_pimpl->descriptors);

    result->keypoints.resize(cvkeypoints.size());
    for (size_t i = 0; i < cvkeypoints.size(); ++i) {
        result->keypoints[i][0] = (cvkeypoints[i].pt.x - m_K(0, 2)) / m_K(0, 0);
        result->keypoints[i][1] = (cvkeypoints[i].pt.y - m_K(1, 2)) / m_K(1, 1);
    }

    return result;
}
