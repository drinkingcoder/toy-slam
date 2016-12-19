#pragma once

#include "Types.h"

namespace slam {

    class RANSAC {
    public:
        size_t iter;
        real score;

        RANSAC(real success_rate = 0.99f, size_t max_iter = 10000000);

        virtual ~RANSAC();

        void run();

    protected:
        virtual size_t data_size() const = 0;
        virtual size_t sample_size() const = 0;

        virtual void reset_model() = 0;
        virtual void fit_model(const std::vector<size_t> &sample_set) = 0;
        virtual real eval_model(std::vector<bool> &inlier_set, size_t &inlier_count) = 0;
        virtual void refine_model(const std::vector<bool> &inlier_set) = 0;

        size_t calc_iter_limit(real success_rate, real inlier_rate, size_t sample_size);

    private:
        real m_success_rate;
        size_t m_max_iter;
    };

}
