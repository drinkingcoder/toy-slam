#include "EightPointEssentialRANSAC.h"
#include "Geometry.h"

using namespace slam;

EightPointEssentialRANSAC::EightPointEssentialRANSAC(const mat3 & K, real sigma, real success_rate, size_t max_iter)
    : RANSAC(success_rate, max_iter), K(K), m_sigma(sigma), E(mat3::Zero())
{
    a.resize(8);
    b.resize(8);
}

EightPointEssentialRANSAC::~EightPointEssentialRANSAC() = default;

void EightPointEssentialRANSAC::set_dataset(const std::vector<vec2>& pa, const std::vector<vec2>& pb, const match_vector & matches) {
    m_ppa = &pa;
    m_ppb = &pb;
    m_pmatches = &matches;
}

size_t EightPointEssentialRANSAC::data_size() const {
    if (m_pmatches) {
        return m_pmatches->size();
    }
    else {
        return 0;
    }
}

size_t EightPointEssentialRANSAC::sample_size() const {
    return 8;
}

void EightPointEssentialRANSAC::reset_model() {
    E = mat3::Zero();
    matches.clear();
    iter = 0;
    score = 0;
}

void EightPointEssentialRANSAC::fit_model(const std::vector<size_t>& sample_set) {
    const std::vector<vec2> &pa = *m_ppa;
    const std::vector<vec2> &pb = *m_ppb;
    const match_vector &matches = *m_pmatches;

    for (size_t i = 0; i < sample_set.size(); ++i) {
        a[i] = pa[matches[sample_set[i]].first];
        b[i] = pb[matches[sample_set[i]].second];
    }
    E = fix_essential(solve_essential(a, b));
}

real EightPointEssentialRANSAC::eval_model(std::vector<bool>& inlier_set, size_t & inlier_count) {
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

void EightPointEssentialRANSAC::refine_model(const std::vector<bool>& inlier_set) {
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

    E = fix_essential(solve_essential(a, b));
}
