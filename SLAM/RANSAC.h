#pragma once

#include "Types.h"
#include "Random.h"
#include "Geometry.h"

namespace slam {

    class RANSAC {
    public:
        size_t iter;
        real score;

        RANSAC(real success_rate = 0.99f, size_t max_iter = 10000000)
            : m_success_rate(success_rate), m_max_iter(max_iter), iter(0), score(0.0f)
        {}

        virtual ~RANSAC() {}

        void run() {
            size_t ds = data_size();
            size_t ss = sample_size();
            if (ds < ss) {
                return;
            }

            size_t iter_limit = m_max_iter;

            LotBox cards(ds);
            std::vector<size_t> sample_set(ss);

            size_t tmp_inlier_count = 0;

            std::vector<bool> tmp_inlier_set(ds);
            std::vector<bool> inlier_set(ds);

            real tmp_score = 0;
            real score = 0;

            for (iter = 0; iter < iter_limit; ++iter) {
                for (size_t k = 0; k < ss; ++k) {
                    sample_set[k] = cards.draw_without_replacement();
                }
                cards.refill_all();

                fit_model(sample_set);

                tmp_score = eval_model(tmp_inlier_set, tmp_inlier_count);

                if (tmp_score > score) {
                    inlier_set.swap(tmp_inlier_set);
                    score = tmp_score;
                    iter_limit = calc_iter_limit(m_success_rate, tmp_inlier_count / (real)ds, ss);
                }
            }

            refine_model(inlier_set);
        }

    protected:
        virtual size_t data_size() const = 0;
        virtual size_t sample_size() const = 0;

        virtual void fit_model(const std::vector<size_t> &sample_set) = 0;
        virtual real eval_model(std::vector<bool> &inlier_set, size_t &inlier_count) = 0;
        virtual void refine_model(const std::vector<bool> &inlier_set) = 0;

        size_t calc_iter_limit(real success_rate, real inlier_rate, size_t sample_size) {
            real N = std::log(1.0f - success_rate) / std::log(1.0f - std::pow(inlier_rate, (real)sample_size));
            if (!std::isfinite(N)) {
                N = 1.0e9f;
            }
            return std::min((size_t)std::ceil(N), m_max_iter);
        }

    private:
        real m_success_rate;
        size_t m_max_iter;
    };

    class EightPointEssentialRANSAC : public RANSAC {
    public:
        mat3 E;
        match_vector matches;

        EightPointEssentialRANSAC(const mat3 &K, real sigma = 1.0f, real success_rate = 0.99f, size_t max_iter = 10000000)
            : RANSAC(success_rate, max_iter), K(K), m_sigma(sigma), E(mat3::Zero())
        {
            a.resize(8);
            b.resize(8);
        }

        void set_dataset(const std::vector<vec2> &pa, const std::vector<vec2> &pb, const match_vector &matches) {
            m_ppa = &pa;
            m_ppb = &pb;
            m_pmatches = &matches;
        }

    protected:
        size_t data_size() const override {
            if (m_pmatches) {
                return m_pmatches->size();
            }
            else {
                return 0;
            }
        }

        size_t sample_size() const override {
            return 8;
        }

        std::vector<vec2> a, b; // optimization purpose
        void fit_model(const std::vector<size_t> &sample_set) override {
            const std::vector<vec2> &pa = *m_ppa;
            const std::vector<vec2> &pb = *m_ppb;
            const match_vector &matches = *m_pmatches;

            for (size_t i = 0; i < sample_set.size(); ++i) {
                a[i] = pa[matches[sample_set[i]].first];
                b[i] = pb[matches[sample_set[i]].second];
            }
            E = fix_essential(solve_essential(a, b));
        }

        real eval_model(std::vector<bool> &inlier_set, size_t &inlier_count) override {
            const std::vector<vec2> &pa = *m_ppa;
            const std::vector<vec2> &pb = *m_ppb;
            const match_vector &matches = *m_pmatches;

            const real chi_square_essential = 3.841f;
            const real chi_square_homography = 5.991f;

            const real inv_sigma_square = 1.0f / (m_sigma*m_sigma);

            real chi_square_score = 0;
            inlier_count = 0;

            for (size_t i = 0; i < matches.size(); ++i) {
                const vec2 &a = pa[matches[i].first];
                const vec2 &b = pb[matches[i].second];
                vec3 l2(E(0, 0)*a(0) + E(0, 1)*a(1) + E(0, 2),
                    E(1, 0)*a(0) + E(1, 1)*a(1) + E(1, 2),
                    E(2, 0)*a(0) + E(2, 1)*a(1) + E(2, 2));
                vec3 l1(E(0, 0)*b(0) + E(1, 0)*b(1) + E(2, 0),
                    E(0, 1)*b(0) + E(1, 1)*b(1) + E(2, 1),
                    E(0, 2)*b(0) + E(1, 2)*b(1) + E(2, 2));

                real d = l2(0)*b(0) + l2(1)*b(1) + l2(2);

                real A1 = l1(0) / K(0, 0);
                real B1 = l1(1) / K(1, 1);
                real A2 = l2(0) / K(0, 0);
                real B2 = l2(1) / K(1, 1);

                real D1_square = d*d / (A1*A1 + B1*B1);
                real D2_square = d*d / (A2*A2 + B2*B2);
                real chi_square_1 = D1_square*inv_sigma_square;
                real chi_square_2 = D2_square*inv_sigma_square;
                if (chi_square_1 < chi_square_essential && chi_square_2 < chi_square_essential) {
                    inlier_set[i] = true;
                    chi_square_score += 2.0f*chi_square_homography - chi_square_1 - chi_square_2;
                    inlier_count++;
                }
                else {
                    inlier_set[i] = false;
                }
            }

            return chi_square_score;
        }

        void refine_model(const std::vector<bool> &inlier_set) override {
            const std::vector<vec2> &pa = *m_ppa;
            const std::vector<vec2> &pb = *m_ppb;
            const match_vector &old_matches = *m_pmatches;

            matches.clear();
            matches.reserve(old_matches.size());
            for (size_t i = 0; i < old_matches.size(); ++i) {
                if (inlier_set[i]) {
                    matches.push_back(old_matches[i]);
                }
            }
            matches.shrink_to_fit();

            std::vector<vec2> a, b;
            a.resize(matches.size());
            b.resize(matches.size());

            for (size_t i = 0; i < matches.size(); ++i) {
                a[i] = pa[matches[i].first];
                b[i] = pb[matches[i].second];
            }

            E = fix_essential(solve_essential(a, b));
        }

    private:
        const std::vector<vec2> *m_ppa = nullptr;
        const std::vector<vec2> *m_ppb = nullptr;
        const match_vector *m_pmatches = nullptr;

        real m_sigma;
        mat3 K;
    };

    class FourPointHomographyRANSAC : public RANSAC {
    public:
        mat3 H;
        match_vector matches;

        FourPointHomographyRANSAC(const mat3 &K, real sigma = 1.0f, real success_rate = 0.99f, size_t max_iter = 10000000)
            : RANSAC(success_rate, max_iter), K(K), m_sigma(sigma), H(mat3::Zero())
        {
            a.resize(4);
            b.resize(4);
        }

        void set_dataset(const std::vector<vec2> &pa, const std::vector<vec2> &pb, const match_vector &matches) {
            m_ppa = &pa;
            m_ppb = &pb;
            m_pmatches = &matches;
        }

    protected:
        size_t data_size() const override {
            if (m_pmatches) {
                return m_pmatches->size();
            }
            else {
                return 0;
            }
        }

        size_t sample_size() const override {
            return 4;
        }

        std::vector<vec2> a, b;
        void fit_model(const std::vector<size_t> &sample_set) override {
            const std::vector<vec2> &pa = *m_ppa;
            const std::vector<vec2> &pb = *m_ppb;
            const match_vector &matches = *m_pmatches;

            for (size_t i = 0; i < sample_set.size(); ++i) {
                a[i] = pa[matches[sample_set[i]].first];
                b[i] = pb[matches[sample_set[i]].second];
            }

            H = solve_homography(a, b);
        }

        real eval_model(std::vector<bool> &inlier_set, size_t &inlier_count) override {
            const std::vector<vec2> &pa = *m_ppa;
            const std::vector<vec2> &pb = *m_ppb;
            const match_vector &matches = *m_pmatches;

            const real chi_square_essential = 3.841f;
            const real chi_square_homography = 5.991f;

            const real inv_sigma_square = 1.0f / (m_sigma*m_sigma);
            const mat3 Hinv = H.inverse();

            real chi_square_score = 0;
            inlier_count = 0;

            for (size_t i = 0; i < matches.size(); ++i) {
                const vec2 &a = pa[matches[i].first];
                const vec2 &b = pb[matches[i].second];
                vec3 Ha(H(0, 0)*a(0) + H(0, 1)*a(1) + H(0, 2),
                    H(1, 0)*a(0) + H(1, 1)*a(1) + H(1, 2),
                    H(2, 0)*a(0) + H(2, 1)*a(1) + H(2, 2));
                vec3 Hinvb(Hinv(0, 0)*b(0) + Hinv(0, 1)*b(1) + Hinv(0, 2),
                    Hinv(1, 0)*b(0) + Hinv(1, 1)*b(1) + Hinv(1, 2),
                    Hinv(2, 0)*b(0) + Hinv(2, 1)*b(1) + Hinv(2, 2));

                real bx = (Ha(0) / Ha(2) - b(0))*K(0, 0);
                real by = (Ha(1) / Ha(2) - b(1))*K(1, 1);
                real ax = (Hinvb(0) / Hinvb(2) - a(0))*K(0, 0);
                real ay = (Hinvb(1) / Hinvb(2) - a(1))*K(1, 1);

                real D1_square = bx*bx + by*by;
                real D2_square = ax*ax + ay*ay;
                real chi_square_1 = D1_square*inv_sigma_square;
                real chi_square_2 = D2_square*inv_sigma_square;
                if (chi_square_1 < chi_square_homography && chi_square_2 < chi_square_homography) {
                    inlier_set[i] = true;
                    chi_square_score += 2.0f*chi_square_homography - chi_square_1 - chi_square_2;
                    inlier_count++;
                }
                else {
                    inlier_set[i] = false;
                }
            }

            return chi_square_score;
        }

        void refine_model(const std::vector<bool> &inlier_set) override {
            const std::vector<vec2> &pa = *m_ppa;
            const std::vector<vec2> &pb = *m_ppb;
            const match_vector &old_matches = *m_pmatches;

            matches.clear();
            matches.reserve(old_matches.size());
            for (size_t i = 0; i < old_matches.size(); ++i) {
                if (inlier_set[i]) {
                    matches.push_back(old_matches[i]);
                }
            }
            matches.shrink_to_fit();

            std::vector<vec2> a, b;
            a.resize(matches.size());
            b.resize(matches.size());

            for (size_t i = 0; i < matches.size(); ++i) {
                a[i] = pa[matches[i].first];
                b[i] = pb[matches[i].second];
            }

            H = solve_homography(a, b);
        }

    private:
        const std::vector<vec2> *m_ppa = nullptr;
        const std::vector<vec2> *m_ppb = nullptr;
        const match_vector *m_pmatches = nullptr;

        real m_sigma;
        mat3 K;
    };

}
