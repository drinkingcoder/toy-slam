#include "Config.h"
#include "CeresMap.h"
#include "Tracker.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include "Initializer.h"
#include "Feature.h"
#include "FourPointPnPRANSAC.h"
#include "Triangulator.h"
#include "UDPSocket.h"

#include <thread>

//void sleep1() {
//    using namespace std::chrono_literals;
//    std::this_thread::sleep_for(1s);
//}
//
//void sleep3() {
//    using namespace std::chrono_literals;
//    std::this_thread::sleep_for(3s);
//}

using namespace slam;

struct ReprojectFunctor {
    ReprojectFunctor(const vec2d &x) : x(x) {}

    template <typename T>
    bool operator() (const T* const p, const T* const r, const T* const t, T *residual) const {
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

        residual[0] = rp[0] / rp[2] - T(x[0]);
        residual[1] = rp[1] / rp[2] - T(x[1]);

        return true;
    }

private:
    const vec2d x;
};

CeresMap::CeresMap(const Config *config) {
    m_pnp = std::make_unique<FourPointPnPRANSAC>(config->K, 1.0f, 0.99f, 200);
    m_triangulator = std::make_unique<Triangulator>(config->K, 1.0f);
    m_K = config->K.cast<double>();
}

CeresMap::~CeresMap() = default;

void slam::CeresMap::clear()
{
    m_keyframes.clear();
    m_landmarks.clear();
    m_last_keyframe.reset();
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

bool CeresMap::init(const std::shared_ptr<Frame> &current_frame, const Initializer *initializer) {

    // reset map
    clear();

    // add

    auto &f1 = initializer->reference_frame;
    auto &f2 = current_frame;
    f1->keyframe_id = add_keyframe(f1);
    f2->keyframe_id = add_keyframe(f2);

    f2->landmark_map.swap(std::vector<size_t>(f2->feature->keypoints.size(), size_t(-1)));
    for (size_t i = 0; i < initializer->points.size(); ++i) {
        size_t lmid = add_landmark(initializer->points[i]);
        f2->landmark_map[initializer->matches[i].second] = lmid;
        add_observation(f1->keyframe_id, lmid, f1->feature->keypoints[initializer->matches[i].first]);
        add_observation(f2->keyframe_id, lmid, f2->feature->keypoints[initializer->matches[i].second]);
    }

    // build problem;
    ceres::Problem problem;

    for (size_t i = 0; i < m_landmarks.size(); ++i) {
        problem.AddParameterBlock(m_landmarks[i].data(), 3);
    }

    ceres::EigenQuaternionParameterization *quatparam = new ceres::EigenQuaternionParameterization();
    ceres::LossFunction *huber = new ceres::HuberLoss(3.0/m_K(0, 0));

    for (size_t i = 0; i < m_keyframes.size(); ++i) {
        problem.AddParameterBlock(m_keyframes[i].rotation.coeffs().data(), 4, quatparam);
        problem.AddParameterBlock(m_keyframes[i].translation.data(), 3);
        problem.SetParameterBlockConstant(m_keyframes[i].translation.data());
        for (auto &ob : m_keyframes[i].observations) {
            ceres::CostFunction *r = new ceres::AutoDiffCostFunction<ReprojectFunctor, 2, 3, 4, 3>(new ReprojectFunctor(ob.second));
            problem.AddResidualBlock(r, huber, m_landmarks[ob.first].data(), m_keyframes[i].rotation.coeffs().data(), m_keyframes[i].translation.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    f2->R = m_keyframes[f2->keyframe_id].rotation.cast<real>().toRotationMatrix();
    f2->T = m_keyframes[f2->keyframe_id].translation.cast<real>();

    // latest frame is stored for matching next
    m_last_keyframe = f2;
    
    send_visualization();
    //sleep3();

    return summary.IsSolutionUsable();
}

bool CeresMap::localize(const std::shared_ptr<Frame>& pframe)
{
    match_vector matches = m_last_keyframe->feature->match(pframe->feature.get(), 5, 0.3f);
    match_vector pnp_matches;
    match_vector image_matches;
    pnp_matches.reserve(matches.size());
    image_matches.reserve(matches.size());

    for (size_t i = 0; i < matches.size(); ++i) {
        size_t mapped_landmark_id = m_last_keyframe->landmark_map[matches[i].first];
        if (mapped_landmark_id != size_t(-1)) {
            pnp_matches.push_back(matches[i]);
            pnp_matches.back().first = mapped_landmark_id;
        }
        else {
            image_matches.push_back(matches[i]);
        }
    }

    m_pnp->set_dataset(m_landmarks, pframe->feature->keypoints, pnp_matches);
    m_pnp->run();

    if (m_pnp->matches.size() < max(pnp_matches.size()/5, 25)) {
        std::cout << "insufficient pnp match" << std::endl;
        return false;
    }

    pnp_matches.swap(m_pnp->matches);

    auto &f1 = m_last_keyframe;
    auto &f2 = pframe;
    f2->R = m_pnp->R;
    f2->T = m_pnp->T;

    vec3 p1 = -f1->R.transpose()*f1->T;
    vec3 p2 = -f2->R.transpose()*f2->T;

    real baseline = (p1 - p2).norm();

    std::cout << "Relative baseline: " << baseline << std::endl;
    if (baseline < 1.0) {
        return true;
    }

    f2->keyframe_id = add_keyframe(f2);
    for (size_t i = 0; i < pnp_matches.size(); ++i) {
        add_observation(f2->keyframe_id, pnp_matches[i].first, f2->feature->keypoints[pnp_matches[i].second]);
    }

    // build problem;
    ceres::Problem problem;

    for (size_t i = 0; i < m_landmarks.size(); ++i) {
        problem.AddParameterBlock(m_landmarks[i].data(), 3);
    }

    ceres::EigenQuaternionParameterization *quatparam = new ceres::EigenQuaternionParameterization();
    ceres::LossFunction *huber = new ceres::HuberLoss(3.0 / m_K(0, 0));

    for (size_t i = 0; i < m_keyframes.size(); ++i) {
        problem.AddParameterBlock(m_keyframes[i].rotation.coeffs().data(), 4, quatparam);
        problem.AddParameterBlock(m_keyframes[i].translation.data(), 3);
        if (i < 2) {
            problem.SetParameterBlockConstant(m_keyframes[i].translation.data());
        }
        for (auto &ob : m_keyframes[i].observations) {
            ceres::CostFunction *r = new ceres::AutoDiffCostFunction<ReprojectFunctor, 2, 3, 4, 3>(new ReprojectFunctor(ob.second));
            problem.AddResidualBlock(r, huber, m_landmarks[ob.first].data(), m_keyframes[i].rotation.coeffs().data(), m_keyframes[i].translation.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        std::cout << "solve fail" << std::endl;
        return false;
    }

    f2->R = m_keyframes[f2->keyframe_id].rotation.cast<real>().toRotationMatrix();
    f2->T = m_keyframes[f2->keyframe_id].translation.cast<real>();

    std::vector<vec3> image_points;

    m_triangulator->set_dataset(m_last_keyframe->feature->keypoints, pframe->feature->keypoints, image_matches);
    m_triangulator->R = f2->R*f1->R.transpose();
    m_triangulator->T = f2->T - m_triangulator->R*f1->T;
    m_triangulator->triangulate();

    image_points.swap(m_triangulator->points);
    image_matches.swap(m_triangulator->matches);

    mat3 RT = m_last_keyframe->R.transpose();
    for (size_t i = 0; i < image_points.size(); ++i) {
        vec3 pt = RT*(image_points[i] - m_last_keyframe->T);
        image_points[i] = pt;
    }

    f2->landmark_map.swap(std::vector<size_t>(f2->feature->keypoints.size(), size_t(-1)));
    for (size_t i = 0; i < pnp_matches.size(); ++i) {
        f2->landmark_map[pnp_matches[i].second] = pnp_matches[i].first;
    }

    for (size_t i = 0; i < image_points.size(); ++i) {
        size_t lmid = add_landmark(image_points[i]);
        f2->landmark_map[image_matches[i].second] = lmid;
        add_observation(f1->keyframe_id, lmid, f1->feature->keypoints[image_matches[i].first]);
        add_observation(f2->keyframe_id, lmid, f2->feature->keypoints[image_matches[i].second]);
    }

    m_last_keyframe = f2;

    send_visualization();

    std::cout << m_keyframes.size() << ": " << m_landmarks.size() << std::endl;

    return true;
}

void CeresMap::send_visualization()
{
    udp::socket socket;
    udp::address dest("127.0.0.1", 6000);

    char buf[1 + sizeof(vec3d)];
    buf[0] = -1;
    socket.send(dest, buf, sizeof(buf));
    buf[0] = 1;
    for (size_t i = 0; i < m_landmarks.size(); ++i) {
        memcpy(buf + 1, m_landmarks[i].data(), sizeof(vec3d));
        socket.send(dest, buf, sizeof(buf));
    }
}

