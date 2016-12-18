#include "OcvImageSequenceStream.h"
#include "OcvImage.h"

using namespace slam;

OcvImageSequenceStream::OcvImageSequenceStream(const std::string & pattern, int begin, int step)
    : m_pattern(pattern), m_current(begin), m_step(step)
{}

OcvImageSequenceStream::~OcvImageSequenceStream() = default;

std::unique_ptr<Image> OcvImageSequenceStream::next()
{
    char buf[256];
    snprintf(buf, 256 * sizeof(char), m_pattern.c_str(), m_current);
    m_current += m_step;
    auto ret = std::make_unique<OcvImage>(buf);
    if (ret->valid()) {
        return ret;
    }
    else {
        return nullptr;
    }
}
