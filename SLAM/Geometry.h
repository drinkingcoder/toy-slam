#pragma once

#include "Types.h"

namespace slam {

    inline mat3 skew_matrix(const vec3 &u) {
        return (mat3() <<
            0, -u.z(), u.y(),
            u.z(), 0, -u.x(),
            -u.y(), u.x(), 0
            ).finished();
    }

    /*
    Solve essential matrix E that pb^T E pa = 0.
    In function, pa and pb are their projective coordinate, assuming the last factor is 1.
    pa and pb should be normalized, if not, use solve_essential.
    number of points must be greater than 8.
    */
    inline mat3 solve_essential_normalized(const std::vector<vec2> &pa, const std::vector<vec2> &pb) {
        matx A;
        A.resize(pa.size(), 9);

        for (size_t i = 0; i < pa.size(); ++i) {
            A(i, 0) = pa[i](0)*pb[i](0);
            A(i, 1) = pa[i](0)*pb[i](1);
            A(i, 2) = pa[i](0);

            A(i, 3) = pa[i](1)*pb[i](0);
            A(i, 4) = pa[i](1)*pb[i](1);
            A(i, 5) = pa[i](1);

            A(i, 6) = pb[i](0);
            A(i, 7) = pb[i](1);
            A(i, 8) = 1;
        }

        vecx e = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(8);
        return Eigen::Map<mat3>(e.data());
    }

    // Solve essential matrix with coordinate normalization
    inline mat3 solve_essential(const std::vector<vec2> &pa, const std::vector<vec2> &pb) {
        vec2 pa_mean = vec2::Zero();
        vec2 pb_mean = vec2::Zero();
        for (size_t i = 0; i < pa.size(); ++i) {
            pa_mean += pa[i];
            pb_mean += pb[i];
        }
        pa_mean /= (real)pa.size();
        pb_mean /= (real)pb.size();

        real sa = 0;
        real sb = 0;
        real sqrt2 = sqrt(2.0f);

        for (size_t i = 0; i < pa.size(); ++i) {
            sa += (pa[i] - pa_mean).norm();
            sb += (pb[i] - pb_mean).norm();
        }

        sa = 1.0f / (sqrt2*sa);
        sb = 1.0f / (sqrt2*sb);

        std::vector<vec2> na(pa.size());
        std::vector<vec2> nb(pb.size());
        for (size_t i = 0; i < pa.size(); ++i) {
            na[i] = (pa[i] - pa_mean)*sa;
            nb[i] = (pb[i] - pb_mean)*sb;
        }

        mat3 E = solve_essential_normalized(na, nb);

        mat3 Na, Nb;
        Nb << sb, 0, 0,
            0, sb, 0,
            -sb*pb_mean(0), -sb*pb_mean(1), 1;
        Na << sa, 0, -sa*pa_mean(0),
            0, sa, -sa*pa_mean(1),
            0, 0, 1;

        E = Nb*E*Na;

        return E;
    }

    /*
    Essential matrix must be a rank-2 matrix with two singular-values equal to 1.
    */
    inline mat3 fix_essential(const mat3 &E) {
        vec3 fs{ 1.0, 1.0, 0.0 };
        Eigen::JacobiSVD<mat3> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        vec3 s = svd.singularValues();
        return svd.matrixU()*fs.asDiagonal()*svd.matrixV().transpose();
    }

    inline vec3 triangulate2(const mat3 &R1, const vec3 &T1, const vec2 &p1, const mat3 &R2, const vec3 &T2, const vec2 &p2) {
        mat4 A;
        A.block<1, 3>(0, 0) = p1(0)*R1.row(2) - R1.row(0);
                    A(0, 3) = p1(0)*T1(2)     - T1(0);
        A.block<1, 3>(1, 0) = p1(1)*R1.row(2) - R1.row(1);
                    A(1, 3) = p1(1)*T1(2)     - T1(1);
        A.block<1, 3>(2, 0) = p2(0)*R2.row(2) - R2.row(0);
                    A(2, 3) = p2(0)*T2(2)     - T2(0);
        A.block<1, 3>(3, 0) = p2(1)*R2.row(2) - R2.row(1);
                    A(3, 3) = p2(1)*T2(2)     - T2(1);

        vec4 x = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
        return x.topLeftCorner<3, 1>() / x(3);
    }

    /*
    Recovering Baseline and Orientation from 'Essential' Matrix, B.K.P. Horn, 1990.
    Each E gives a pair of R, T. However notice that E and -E is indistinguisable just
    from epipolar constraint, hence there are 4 groups of solutions.
    Among the 4, only one solution where points are in front of both cameras.
    */
    inline void decompose_essential(const mat3 &E, mat3 &R1, mat3 &R2, vec3 &T1, vec3 &T2) {
        mat3 EET = E*E.transpose();
        real halfTrace = 0.5f*EET.trace();
        vec3 b;

        vec3 e0e1 = E.col(0).cross(E.col(1));
        vec3 e1e2 = E.col(1).cross(E.col(2));
        vec3 e2e0 = E.col(2).cross(E.col(0));

    #if 0
        mat3 bbT = halfTrace*mat3::Identity() - EET;
        vec3 bbT_diag = bbT.diagonal();
        if (bbT_diag(0) > bbt_diag(1) && bbT_diag(0) > bbT_diag(2)) {
            b = bbT.row(0) / sqrt(bbT_diag(0));
        }
        else if (bbT_diag(1) > bbT_diag(0) && bbT_diag(1) > bbT_diag(2)) {
            b = bbT.row(1) / sqrt(bbT_diag(1));
        }
        else {
            b = bbT.row(2) / sqrt(bbT_diag(2));
        }
    #else
        if (e0e1.norm() > e1e2.norm() && e0e1.norm() > e2e0.norm()) {
            b = e0e1.normalized()*sqrt(halfTrace);
        }
        else if (e1e2.norm() > e0e1.norm() && e1e2.norm() > e2e0.norm()) {
            b = e1e2.normalized()*sqrt(halfTrace);
        }
        else {
            b = e2e0.normalized()*sqrt(halfTrace);
        }
    #endif

        mat3 cofactorsT;
        cofactorsT.col(0) = e1e2;
        cofactorsT.col(1) = e2e0;
        cofactorsT.col(2) = e0e1;

        R1 = (cofactorsT - skew_matrix(b)*E) / b.dot(b);
        T1 = b;
        R2 = (cofactorsT + skew_matrix(b)*E) / b.dot(b);
        T2 = -b;
    }

    /*
    Solve homography matrix H that pb x H pa = 0.
    In function, pa and pb are their projective coordinate, assuming the last factor is 1.
    pa and pb should be normalized, if not, use solve_homography.
    number of points must be greater than 4.
    */
    inline mat3 solve_homography_normalized(const std::vector<vec2> &pa, const std::vector<vec2> &pb) {
        matx A;
        A.resize(pa.size() * 2, 9);
        A.setZero();

        for (size_t i = 0; i < pa.size(); ++i) {
            const vec2 &a = pa[i];
            const vec2 &b = pb[i];
            A(i * 2, 1) = -a(0);
            A(i * 2, 2) = a(0)*b(1);
            A(i * 2, 4) = -a(1);
            A(i * 2, 5) = a(1)*b(1);
            A(i * 2, 7) = -1;
            A(i * 2, 8) = b(1);
            A(i * 2 + 1, 0) = a(0);
            A(i * 2 + 1, 2) = -a(0)*b(0);
            A(i * 2 + 1, 3) = a(1);
            A(i * 2 + 1, 5) = -a(1)*b(0);
            A(i * 2 + 1, 6) = 1;
            A(i * 2 + 1, 8) = -b(0);
        }

        vecx h = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(8);
        return Eigen::Map<mat3>(h.data());
    }

    // Solve homography matrix with coordinate normalization.
    inline mat3 solve_homography(const std::vector<vec2> &pa, const std::vector<vec2> &pb) {
        vec2 pa_mean = vec2::Zero();
        vec2 pb_mean = vec2::Zero();
        for (size_t i = 0; i < pa.size(); ++i) {
            pa_mean += pa[i];
            pb_mean += pb[i];
        }
        pa_mean /= (real)pa.size();
        pb_mean /= (real)pb.size();

        real sa = 0;
        real sb = 0;
        real sqrt2 = sqrt(2.0f);

        for (size_t i = 0; i < pa.size(); ++i) {
            sa += (pa[i] - pa_mean).norm();
            sb += (pb[i] - pb_mean).norm();
        }

        sa = 1.0f / (sqrt2*sa);
        sb = 1.0f / (sqrt2*sb);

        std::vector<vec2> na(pa.size());
        std::vector<vec2> nb(pb.size());
        for (size_t i = 0; i < pa.size(); ++i) {
            na[i] = (pa[i] - pa_mean)*sa;
            nb[i] = (pb[i] - pb_mean)*sb;
        }

        mat3 H = solve_homography_normalized(na, nb);

        mat3 Na, Nb;
        Nb << 1 / sb, 0, pb_mean(0),
            0, 1 / sb, pb_mean(1),
            0, 0, 1;
        Na << sa, 0, -sa*pa_mean(0),
            0, sa, -sa*pa_mean(1),
            0, 0, 1;

        H = Nb*H*Na;

        return H;
    }

    inline vec2 project(const vec3 &p) {
        return p.topLeftCorner<2, 1>() / p.z();
    }

}
