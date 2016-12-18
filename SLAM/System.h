#pragma once

#include <memory>

namespace slam {

    class ImageStream;
    class Tracker;

    class System {
    public:
        System();
        virtual ~System();

        int run();

    private:
        std::unique_ptr<ImageStream> m_stream;
        std::unique_ptr<Tracker> m_tracker;
    };

}
