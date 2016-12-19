#pragma once

#include <memory>
#include "Types.h"
#include "Feature.h"

namespace slam {

    struct OcvOrbFeature_Impl;

    class OcvOrbFeature : public Feature {
        friend struct OcvHelperFunctions;
    public:
        OcvOrbFeature();
        ~OcvOrbFeature();

        match_vector match(const Feature *feature, size_t k = 5, real radius = 1.0e7f) const override;

    private:
        friend class OcvOrbFeatureExtractor;
        std::unique_ptr<OcvOrbFeature_Impl> m_pimpl;
    };

    struct OcvOrbFeatureExtractor_Impl;
    class Config;

    class OcvOrbFeatureExtractor : public FeatureExtractor {
    public:
        OcvOrbFeatureExtractor(const Config *config);
        ~OcvOrbFeatureExtractor();

        std::unique_ptr<Feature> extract(const Image *image) const override;

    private:
        std::unique_ptr<OcvOrbFeatureExtractor_Impl> m_pimpl;
        mat3 m_K;
        int m_spread_size;
    };

}
