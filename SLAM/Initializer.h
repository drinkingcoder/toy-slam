#pragma once

namespace slam {

    class Tracker;

    class Initializer {
    public:
        virtual ~Initializer() {}

        virtual void initialize(Tracker *tracker) = 0;

    };

}
