#pragma once

#include <algorithm>
#include <future>
#include "Types.h"
#include "Geometry.h"

namespace slam {

    class Triangulator {
    public:
        mat3 R;
        vec3 T;
        std::vector<vec3> points;
        std::vector<std::pair<size_t, size_t>> matches;
        real parallax;

        Triangulator(const mat3 &K, real sigma);
        ~Triangulator();

        void set_dataset(const std::vector<vec2> &pa, const std::vector<vec2> &pb, const std::vector<std::pair<size_t, size_t>> &matches);

        void run(const mat3 &E);

        void triangulate();

    private:
        size_t try_triangulate(const mat3 &R, const vec3 &T, std::vector<vec3> &triangulated, std::vector<bool> &inlier_set, real &rparallax) const;

        const std::vector<vec2> *m_ppa = nullptr;
        const std::vector<vec2> *m_ppb = nullptr;
        const std::vector<std::pair<size_t, size_t>> *m_pmatches = nullptr;

        mat3 K;
        real m_sigma2;
    };

}
