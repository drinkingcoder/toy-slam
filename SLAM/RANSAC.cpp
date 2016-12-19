#include "RANSAC.h"
#include "Random.h"

using namespace slam;

RANSAC::RANSAC(real success_rate, size_t max_iter)
    : m_success_rate(success_rate), m_max_iter(max_iter), iter(0), score(0.0f)
{}

RANSAC::~RANSAC() = default;

void RANSAC::run() {
    size_t ds = data_size();
    size_t ss = sample_size();
    if (ds < ss) {
        reset_model();
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

size_t RANSAC::calc_iter_limit(real success_rate, real inlier_rate, size_t sample_size) {
    real N = std::log(1.0f - success_rate) / std::log(1.0f - std::pow(inlier_rate, (real)sample_size));
    if (!std::isfinite(N)) {
        N = 1.0e9f;
    }
    return std::min((size_t)std::ceil(N), m_max_iter);
}
