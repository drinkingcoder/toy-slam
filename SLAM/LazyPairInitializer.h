#pragma once

#include <memory>
#include "Initializer.h"

namespace slam {

    class EightPointEssentialRANSAC;
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
        std::unique_ptr<EightPointEssentialRANSAC> m_essential_ransac;
    };

}
