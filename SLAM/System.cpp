#include "System.h"
#include "OcvYamlConfig.h"
#include "OcvImageSequenceStream.h"
#include "OcvCameraImageStream.h"
#include "Tracker.h"

using namespace slam;

System::System() {
    m_config = std::make_unique<OcvYamlConfig>("config.yaml");

    std::string input_type = m_config->text("Input.type", "Camera");
    if (input_type == "camera") {
        m_stream = std::make_unique<OcvCameraImageStream>();
    }
    else if (input_type == "sequence") {
        m_stream = std::make_unique<OcvImageSequenceStream>(
            m_config->text("Input.Sequence.pattern", "", false),
            (int)m_config->value("Input.Sequence.begin"),
            (int)m_config->value("Input.Sequence.step", 1)
        );
    }

    m_tracker = std::make_unique<Tracker>(m_config.get());
}

System::~System() = default;

int System::run() {
    while (auto image = m_stream->next()) {
        m_tracker->track(image.get());
    }
    return 0;
}
