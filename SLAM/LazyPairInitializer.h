#pragma once

#include <memory>
#include "Initializer.h"
#include "Types.h"

namespace slam {

    class EightPointEssentialRANSAC;
    class FourPointHomographyRANSAC;
    class Triangulator;
    class Frame;
    class Config;

    class Image;

    class LazyPairInitializer : public Initializer {
    public:
        LazyPairInitializer(const Config *config);
        ~LazyPairInitializer();

        virtual bool initialize(Tracker *tracker);

    private:
        Frame *m_first_frame = nullptr;
        size_t m_frame_count = 0;
        real m_min_parallax;
        mat3 m_K;
        std::unique_ptr<EightPointEssentialRANSAC> m_essential_ransac;
        std::unique_ptr<FourPointHomographyRANSAC> m_homography_ransac;
        std::unique_ptr<Triangulator> m_triangulator;
    };

}
