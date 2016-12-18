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

        enum TriangulationState { STATE_SUCCESS, STATE_AMBIGUIOUS, STATE_SMALL_PARALLAX, STATE_TERMINATED } state;

        Triangulator(const mat3 &K, real sigma, real min_parallax_angle, size_t min_triangulated)
            : K(K), m_sigma2(sigma*sigma), m_cos_parallax_limit(cos(min_parallax_angle*(real)(3.14159265359 / 180.0))), m_min_triangulated(min_triangulated)
        {
            R = mat3::Identity();
            T = vec3::Zero();
        }

        void set_dataset(const std::vector<vec2> &pa, const std::vector<vec2> &pb, const std::vector<std::pair<size_t, size_t>> &matches) {
            m_ppa = &pa;
            m_ppb = &pb;
            m_pmatches = &matches;
        }

        void run(const mat3 &E) {
            R = mat3::Identity();
            T = vec3::Zero();
            points.clear();
            matches.clear();

            mat3 R1, R2; vec3 T1, T2;
            decompose_essential(E, R1, R2, T1, T2);

            size_t N = m_pmatches->size();
            std::vector<vec3> points1(N), points2(N), points3(N), points4(N);
            std::vector<bool> inliers1(N, false), inliers2(N, false), inliers3(N, false), inliers4(N, false);

#ifdef USE_PARALLEL_TRIANGULATION
            std::future<size_t> t1 = std::async(std::launch::async, &Triangulator::try_triangulate, this, R1, T1, std::ref(points1), std::ref(inliers1)/*, std::cref(terminate_signal)*/);
            std::future<size_t> t2 = std::async(std::launch::async, &Triangulator::try_triangulate, this, R2, T1, std::ref(points2), std::ref(inliers2)/*, std::cref(terminate_signal)*/);
            std::future<size_t> t3 = std::async(std::launch::async, &Triangulator::try_triangulate, this, R1, T2, std::ref(points3), std::ref(inliers3)/*, std::cref(terminate_signal)*/);
            std::future<size_t> t4 = std::async(std::launch::async, &Triangulator::try_triangulate, this, R2, T2, std::ref(points4), std::ref(inliers4)/*, std::cref(terminate_signal)*/);

            size_t count1 = t1.get();
            size_t count2 = t2.get();
            size_t count3 = t3.get();
            size_t count4 = t4.get();
#else
            size_t count1 = try_triangulate(R1, T1, points1, inliers1);
            size_t count2 = try_triangulate(R2, T1, points2, inliers2);
            size_t count3 = try_triangulate(R1, T2, points3, inliers3);
            size_t count4 = try_triangulate(R2, T2, points4, inliers4);
#endif

            size_t max_count = std::max(std::max(count1, count2), std::max(count3, count4));

            if (max_count < std::max(N / 4, m_min_triangulated)) {
                // Insufficient matches, this failure is because too small parallax
                state = STATE_SMALL_PARALLAX;
                return;
            }

            size_t similar_count = 0;
            similar_count += ((count1 * 4 > max_count * 3) ? 1 : 0);
            similar_count += ((count2 * 4 > max_count * 3) ? 1 : 0);
            similar_count += ((count3 * 4 > max_count * 3) ? 1 : 0);
            similar_count += ((count4 * 4 > max_count * 3) ? 1 : 0);

            if (similar_count != 1) {
                // Ambiguious
                state = STATE_AMBIGUIOUS;
                return;
            }

            std::vector<vec3> *ppoints = nullptr;
            std::vector<bool> *pinliers = nullptr;
            if (max_count == count1) {
                R = R1; T = T1;
                ppoints = &points1;
                pinliers = &inliers1;
            }
            else if (max_count == count2) {
                R = R2; T = T1;
                ppoints = &points2;
                pinliers = &inliers2;
            }
            else if (max_count == count3) {
                R = R1; T = T2;
                ppoints = &points3;
                pinliers = &inliers3;
            }
            else if (max_count == count4) {
                R = R2; T = T2;
                ppoints = &points4;
                pinliers = &inliers4;
            }
            std::vector<vec3> &rpoints = *ppoints;
            std::vector<bool> &rinliers = *pinliers;
            const std::vector<std::pair<size_t, size_t>> &rmatches = *m_pmatches;
            points.reserve(N);
            matches.reserve(N);
            for (size_t i = 0; i < N; ++i) {
                if (rinliers[i]) {
                    points.push_back(rpoints[i]);
                    matches.push_back(rmatches[i]);
                }
            }
            points.shrink_to_fit();
            matches.shrink_to_fit();

            state = STATE_SUCCESS;
        }

        void refine() {

        }

    private:
        size_t try_triangulate(const mat3 &R, const vec3 &T, std::vector<vec3> &triangulated, std::vector<bool> &inlier_set/*, const std::atomic<bool> &terminate_signal*/) const {
            const std::vector<vec2> &pa = *m_ppa;
            const std::vector<vec2> &pb = *m_ppb;
            const std::vector<std::pair<size_t, size_t>> &rmatches = *m_pmatches;

            size_t good_count = 0;

            vec3 C2 = -R.transpose()*T;

            for (size_t i = 0; i < rmatches.size(); ++i) {
                const vec2 &p1 = pa[rmatches[i].first];
                const vec2 &p2 = pb[rmatches[i].second];

                vec3 P1 = triangulate2(mat3::Identity(), vec3::Zero(), p1, R, T, p2);

                if (!P1.allFinite()) {
                    continue;
                }

                vec3 P2 = R*P1 + T;

                vec3 n1 = P1.normalized();
                vec3 n2 = (P1 - C2).normalized();

                if (n1.dot(n2) > m_cos_parallax_limit) {
                    continue;
                }

                if (P1.z() <= 0 || P2.z() <= 0) {
                    continue;
                }

                vec2 diff1 = project(P1) - p1;
                vec2 diff2 = project(P2) - p2;

                diff1.x() *= K(0, 0);
                diff1.y() *= K(1, 1);
                diff2.x() *= K(0, 0);
                diff2.y() *= K(1, 1);

                if (diff1.squaredNorm() > m_sigma2 || diff2.squaredNorm() > m_sigma2) {
                    continue;
                }

                triangulated[i] = P1;
                inlier_set[i] = true;
                good_count++;
            }

            return good_count;
        }

        const std::vector<vec2> *m_ppa = nullptr;
        const std::vector<vec2> *m_ppb = nullptr;
        const std::vector<std::pair<size_t, size_t>> *m_pmatches = nullptr;

        mat3 K;
        real m_sigma2;
        real m_cos_parallax_limit;
        size_t m_min_triangulated;
    };

}
