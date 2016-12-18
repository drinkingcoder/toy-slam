#include "System.h"
#include "OcvImageSequenceStream.h"
#include "Tracker.h"

using namespace slam;

System::System() {
    m_stream = std::make_unique<OcvImageSequenceStream>("E:\\Gangwan-street\\1\\rect%05d.jpg", 0);
    m_tracker = std::make_unique<Tracker>();
}

System::~System() = default;

int System::run() {
    while (auto image = m_stream->next()) {
        m_tracker->track(image.get());
    }
    return 0;
}
