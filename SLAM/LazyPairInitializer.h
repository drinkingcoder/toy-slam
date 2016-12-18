#pragma once

#include <memory>
#include "Initializer.h"

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

        virtual void initialize(Tracker *tracker);

    private:
        Frame *m_first_frame = nullptr;
        size_t m_frame_count = 0;
        std::unique_ptr<EightPointEssentialRANSAC> m_essential_ransac;
        std::unique_ptr<FourPointHomographyRANSAC> m_homography_ransac;
        std::unique_ptr<Triangulator> m_triangulator;
    };

}
