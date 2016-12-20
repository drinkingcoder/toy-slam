#include "CeresMap.h"
#include "Tracker.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>

using namespace slam;

struct ReprojectFunctor {
    template <typename T>
    bool operator() (const T* const x, const T* const p, const T* const r, const T* const t, T *residual) const {
        T q[4];
        q[0] = r[3]; // w
        q[1] = r[0]; // x
        q[2] = r[1]; // y
        q[3] = r[2]; // z

        T rp[3];
        ceres::QuaternionRotatePoint(q, p, rp);

        rp[0] += t[0];
        rp[1] += t[1];
        rp[2] += t[2];

        residual[0] = rp[0] / rp[2] - x[0];
        residual[1] = rp[1] / rp[2] - x[1];

        return true;
    }
};

CeresMap::CeresMap() = default;

CeresMap::~CeresMap() = default;

void slam::CeresMap::clear()
{
    m_keyframes.clear();
    m_landmarks.clear();
}

size_t CeresMap::add_keyframe(const std::shared_ptr<Frame> &pframe) {
    size_t id = m_keyframes.size();
    m_keyframes.push_back(Pose());
    m_keyframes[id].rotation = pframe->R.cast<double>();
    m_keyframes[id].translation = pframe->T.cast<double>();
    return id;
}

size_t CeresMap::add_landmark(const vec3 &point) {
    size_t id = m_landmarks.size();
    m_landmarks.push_back(point.cast<double>());
    return id;
}

void CeresMap::add_observation(size_t keyframe, size_t landmark, const vec2 & x) {
    m_keyframes[keyframe].observations[landmark] = x.cast<double>();
}

bool CeresMap::init(size_t keyframe1, size_t keyframe2) {
    // build problem;
    ceres::Problem problem;

    for (size_t i = 0; i < m_landmarks.size(); ++i) {
        problem.AddParameterBlock(m_landmarks[i].data(), 3);
    }

    ceres::EigenQuaternionParameterization *quatparam = new ceres::EigenQuaternionParameterization();
    //ceres::LossFunction *huber = new ceres::HuberLossFunction()

    for (size_t i = 0; i < m_keyframes.size(); ++i) {
        problem.AddParameterBlock(m_keyframes[i].rotation.coeffs().data(), 4, quatparam);
        problem.AddParameterBlock(m_keyframes[i].translation.data(), 3);
        problem.SetParameterBlockConstant(m_keyframes[i].translation.data());
        for (auto &ob : m_keyframes[i].observations) {
            ceres::CostFunction *r = new ceres::AutoDiffCostFunction<ReprojectFunctor, 2, 2, 3, 4, 3>(new ReprojectFunctor);
            problem.AddResidualBlock(r, nullptr, ob.second.data(), m_landmarks[ob.first].data(), m_keyframes[i].rotation.coeffs().data(), m_keyframes[i].translation.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return summary.IsSolutionUsable();
}
