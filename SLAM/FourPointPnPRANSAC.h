#pragma once

#include "RANSAC.h"

namespace slam {

    class FourPointPnPRANSAC : public RANSAC {
    public:
        mat3 R;
        vec3 T;
        match_vector matches;

        FourPointPnPRANSAC(const mat3 &K, real sigma = 1.0f, real success_rate = 0.99f, size_t max_iter = 10000000);
        ~FourPointPnPRANSAC();

        void set_dataset(const std::vector<vec3d> &pa, const std::vector<vec2> &pb, const match_vector &matches);

    protected:
        size_t data_size() const override;

        size_t sample_size() const override;

        void reset_model() override;

        void fit_model(const std::vector<size_t> &sample_set) override;

        real eval_model(std::vector<bool> &inlier_set, size_t &inlier_count) override;

        void refine_model(const std::vector<bool> &inlier_set) override;

    private:
        const std::vector<vec3d> *m_ppa = nullptr;
        const std::vector<vec2> *m_ppb = nullptr;
        const match_vector *m_pmatches = nullptr;

        real m_sigma;
        mat3 K;
    };

}
