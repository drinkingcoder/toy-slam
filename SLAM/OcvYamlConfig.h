#pragma once

#include <memory>
#include "Config.h"

namespace slam {

    struct OcvYamlConfig_Impl;

    class OcvYamlConfig : public Config {
    public:
        OcvYamlConfig(const std::string &filepath);
        ~OcvYamlConfig();

        std::string text(const std::string &config, const std::string &def = "") const override;
        double value(const std::string &config, const double &def = 0.0) const override;

    private:
        std::unique_ptr<OcvYamlConfig_Impl> m_pimpl;
    };

}
