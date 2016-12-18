#pragma once

#include <string>

namespace slam {

    class Config {
    public:
        virtual ~Config() {}

        virtual std::string text(const std::string &config, const std::string &def = "") const = 0;
        virtual double value(const std::string &config, const double &def = 0.0) const  = 0;

    };

}
