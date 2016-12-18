#include "Initializer.h"
#include "Tracker.h"
#include "Feature.h"

#include <iostream>

using namespace slam;

LazyPairInitializer::~LazyPairInitializer() = default;

void LazyPairInitializer::initialize(Tracker *tracker) {
    if (m_first_frame) {
        auto matches = tracker->current_frame().feature->match(m_first_frame->feature.get());
        std::cout << matches.size() << std::endl;
    }
    else {
        m_first_frame = &(tracker->current_frame());
    }
}
