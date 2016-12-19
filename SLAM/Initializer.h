#pragma once

namespace slam {

    class Tracker;

    class Initializer {
    public:
        virtual ~Initializer() {}

        virtual bool initialize(Tracker *tracker) = 0;

    };

}
