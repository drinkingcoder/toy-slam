#pragma once

#include <unordered_map>
#include "Map.h"

//namespace ceres {
//    class Problem;
//}

namespace slam {
    
    class CeresMap : public Map {
    public:
        CeresMap();
        ~CeresMap();

        void clear() override;

        size_t add_keyframe(const std::shared_ptr<Frame> &pframe) override;
        size_t add_landmark(const vec3 &point) override;

        void add_observation(size_t keyframe, size_t landmark, const vec2 &x) override;

        bool init(size_t keyframe1, size_t keyframe2) override;

    private:
        struct Pose {
            quatd rotation;
            vec3d translation;
            std::unordered_map<size_t, vec2d> observations;
        };

        std::vector<Pose> m_keyframes;
        std::vector<vec3d> m_landmarks;

        //std::unique_ptr<ceres::Problem> m_problem;
    };

}