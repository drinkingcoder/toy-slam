#pragma once

namespace slam {

    class Image {
    public:
        virtual ~Image() {}

        virtual bool valid() const { return false; }

    };

}
