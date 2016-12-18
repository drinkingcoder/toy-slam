#pragma once

#include <memory>

namespace slam {

    class Config;
    class ImageStream;
    class Tracker;

    class System {
    public:
        System();
        virtual ~System();

        int run();

    private:
        std::unique_ptr<Config> m_config;
        std::unique_ptr<ImageStream> m_stream;
        std::unique_ptr<Tracker> m_tracker;
    };

}
