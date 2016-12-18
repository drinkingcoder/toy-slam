#pragma once

namespace slam {

    class Tracker;
    class Frame;

    class Initializer {
    public:
        virtual ~Initializer() {}

        virtual void initialize(Tracker *tracker) = 0;

    };

    class LazyPairInitializer : public Initializer {
    public:
        ~LazyPairInitializer();

        virtual void initialize(Tracker *tracker);

    private:
        Frame *m_first_frame = nullptr;
    };

}
