#include "FourPointHomographyRANSAC.h"
#include "Geometry.h"

using namespace slam;

FourPointHomographyRANSAC::FourPointHomographyRANSAC(const mat3 & K, real sigma, real success_rate, size_t max_iter)
    : RANSAC(success_rate, max_iter), K(K), m_sigma(sigma), H(mat3::Zero())
{
    a.resize(4);
    b.resize(4);
}

FourPointHomographyRANSAC::~FourPointHomographyRANSAC() = default;

void FourPointHomographyRANSAC::set_dataset(const std::vector<vec2>& pa, const std::vector<vec2>& pb, const match_vector & matches) {
    m_ppa = &pa;
    m_ppb = &pb;
    m_pmatches = &matches;
}

size_t FourPointHomographyRANSAC::data_size() const {
    if (m_pmatches) {
        return m_pmatches->size();
    }
    else {
        return 0;
    }
}

size_t FourPointHomographyRANSAC::sample_size() const {
    return 4;
}

void FourPointHomographyRANSAC::reset_model() {
    H = mat3::Zero();
    matches.clear();
    iter = 0;
    score = 0;
}

void FourPointHomographyRANSAC::fit_model(const std::vector<size_t>& sample_set) {
    const std::vector<vec2> &pa = *m_ppa;
    const std::vector<vec2> &pb = *m_ppb;
    const match_vector &matches = *m_pmatches;

    for (size_t i = 0; i < sample_set.size(); ++i) {
        a[i] = pa[matches[sample_set[i]].first];
        b[i] = pb[matches[sample_set[i]].second];
    }

    H = solve_homography(a, b);
}

real FourPointHomographyRANSAC::eval_model(std::vector<bool>& inlier_set, size_t & inlier_count) {
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

void FourPointHomographyRANSAC::refine_model(const std::vector<bool>& inlier_set) {
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

    if (matches.size() < sample_size()) {
        return;
    }

    std::vector<vec2> a, b;
    a.resize(matches.size());
    b.resize(matches.size());

    for (size_t i = 0; i < matches.size(); ++i) {
        a[i] = pa[matches[i].first];
        b[i] = pb[matches[i].second];
    }

    H = solve_homography(a, b);
}
