#pragma once

#include <string>
#include "Types.h"

namespace slam {

    class Config {
    public:
        Config() { K = mat3::Identity(); }
        virtual ~Config() {}

        virtual std::string text(const std::string &config, const std::string &def = "", bool normalize = true) const = 0;
        virtual double value(const std::string &config, const double &def = 0.0) const  = 0;

        mat3 K;

    };

}
