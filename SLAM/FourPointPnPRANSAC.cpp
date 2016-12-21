#include "FourPointPnPRANSAC.h"
#include "Geometry.h"
#include <opencv2/opencv.hpp>

using namespace slam;

FourPointPnPRANSAC::FourPointPnPRANSAC(const mat3 & K, real sigma, real success_rate, size_t max_iter)
    : RANSAC(success_rate, max_iter), K(K), m_sigma(sigma), R(mat3::Identity()), T(vec3::Zero())
{}

FourPointPnPRANSAC::~FourPointPnPRANSAC() = default;

void FourPointPnPRANSAC::set_dataset(const std::vector<vec3d>& pa, const std::vector<vec2>& pb, const match_vector & matches) {
    m_ppa = &pa;
    m_ppb = &pb;
    m_pmatches = &matches;
}

size_t FourPointPnPRANSAC::data_size() const {
    if (m_pmatches) {
        return m_pmatches->size();
    }
    else {
        return 0;
    }
}

size_t FourPointPnPRANSAC::sample_size() const {
    return 4;
}

void FourPointPnPRANSAC::reset_model() {
    R = mat3::Identity();
    T = vec3::Zero();
    matches.clear();
    iter = 0;
    score = 0;
}

void FourPointPnPRANSAC::fit_model(const std::vector<size_t>& sample_set) {
    const std::vector<vec3d> &pa = *m_ppa;
    const std::vector<vec2> &pb = *m_ppb;
    const match_vector &matches = *m_pmatches;

    std::vector<cv::Point3f> world_point(sample_set.size());
    std::vector<cv::Point2f> image_point(sample_set.size());
    for (size_t i = 0; i < sample_set.size(); ++i) {
        world_point[i].x = (float)pa[matches[sample_set[i]].first].x();
        world_point[i].y = (float)pa[matches[sample_set[i]].first].y();
        world_point[i].z = (float)pa[matches[sample_set[i]].first].z();
        image_point[i].x = pb[matches[sample_set[i]].second].x();
        image_point[i].y = pb[matches[sample_set[i]].second].y();
    }

    cv::Mat rvec, tvec, rmat;
    cv::solvePnP(world_point, image_point, cv::Mat::eye(3, 3, CV_32FC1), cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_P3P);
    cv::Rodrigues(rvec, rmat);
    R(0, 0) = (real)rmat.at<double>(0, 0);
    R(0, 1) = (real)rmat.at<double>(0, 1);
    R(0, 2) = (real)rmat.at<double>(0, 2);
    R(1, 0) = (real)rmat.at<double>(1, 0);
    R(1, 1) = (real)rmat.at<double>(1, 1);
    R(1, 2) = (real)rmat.at<double>(1, 2);
    R(2, 0) = (real)rmat.at<double>(2, 0);
    R(2, 1) = (real)rmat.at<double>(2, 1);
    R(2, 2) = (real)rmat.at<double>(2, 2);
    T(0) = (real)tvec.at<double>(0);
    T(1) = (real)tvec.at<double>(1);
    T(2) = (real)tvec.at<double>(2);
}

real FourPointPnPRANSAC::eval_model(std::vector<bool>& inlier_set, size_t & inlier_count) {
    const std::vector<vec3d> &pa = *m_ppa;
    const std::vector<vec2> &pb = *m_ppb;
    const match_vector &matches = *m_pmatches;

    const real chi_square_essential = 3.841f;
    const real chi_square_homography = 5.991f;

    const real inv_sigma_square = 1.0f / (m_sigma*m_sigma);

    real chi_square_score = 0;
    inlier_count = 0;

    for (size_t i = 0; i < matches.size(); ++i) {
        vec3 a = pa[matches[i].first].cast<real>();
        const vec2 &b = pb[matches[i].second];

        vec3 p = R*a + T;
        vec2 diff = (p.topLeftCorner<2, 1>() / p(2)) - b;
        real diff_x = diff.x()*K(0, 0);
        real diff_y = diff.y()*K(1, 1);
        real chi_square = (diff_x*diff_x + diff_y*diff_y)*inv_sigma_square;

        if (chi_square < chi_square_homography) {
            inlier_set[i] = true;
            chi_square_score += chi_square_homography - chi_square;
            inlier_count++;
        }
        else {
            inlier_set[i] = false;
        }
    }

    return chi_square_score;
}

void FourPointPnPRANSAC::refine_model(const std::vector<bool>& inlier_set) {
    const std::vector<vec3d> &pa = *m_ppa;
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

    std::vector<cv::Point3f> world_point(matches.size());
    std::vector<cv::Point2f> image_point(matches.size());

    for (size_t i = 0; i < matches.size(); ++i) {
        world_point[i].x = (float)pa[matches[i].first].x();
        world_point[i].y = (float)pa[matches[i].first].y();
        world_point[i].z = (float)pa[matches[i].first].z();
        image_point[i].x = pb[matches[i].second].x();
        image_point[i].y = pb[matches[i].second].y();
    }

    cv::Mat rvec, tvec, rmat;
    cv::solvePnP(world_point, image_point, cv::Mat::eye(3, 3, CV_32FC1), cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_EPNP);
    cv::Rodrigues(rvec, rmat);
    R(0, 0) = (real)rmat.at<double>(0, 0);
    R(0, 1) = (real)rmat.at<double>(0, 1);
    R(0, 2) = (real)rmat.at<double>(0, 2);
    R(1, 0) = (real)rmat.at<double>(1, 0);
    R(1, 1) = (real)rmat.at<double>(1, 1);
    R(1, 2) = (real)rmat.at<double>(1, 2);
    R(2, 0) = (real)rmat.at<double>(2, 0);
    R(2, 1) = (real)rmat.at<double>(2, 1);
    R(2, 2) = (real)rmat.at<double>(2, 2);
    T(0) = (real)tvec.at<double>(0);
    T(1) = (real)tvec.at<double>(1);
    T(2) = (real)tvec.at<double>(2);
}
