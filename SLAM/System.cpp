#include "System.h"
#include "OcvYamlConfig.h"
#include "OcvImageSequenceStream.h"
#include "Tracker.h"

using namespace slam;

System::System() {
    m_config = std::make_unique<OcvYamlConfig>("config.yaml");

    m_stream = std::make_unique<OcvImageSequenceStream>(
        m_config->text("Input.sequence.pattern"),
        (int)m_config->value("Input.sequence.begin"),
        (int)m_config->value("Input.sequence.step", 1)
    );

    m_tracker = std::make_unique<Tracker>(m_config.get());
}

System::~System() = default;

int System::run() {
    while (auto image = m_stream->next()) {
        m_tracker->track(image.get());
    }
    return 0;
}
