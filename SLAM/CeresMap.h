#pragma once

#include <unordered_map>
#include "Map.h"

namespace slam {

    class Config;
    class FourPointPnPRANSAC;
    class Triangulator;

    class CeresMap : public Map {
    public:
        CeresMap(const Config *config);
        ~CeresMap();

        void clear() override;

        size_t add_keyframe(const std::shared_ptr<Frame> &pframe) override;
        size_t add_landmark(const vec3 &point) override;

        void add_observation(size_t keyframe, size_t landmark, const vec2 &x) override;

        bool init(const std::shared_ptr<Frame> &current_frame, const Initializer *initializer) override;

        bool localize(const std::shared_ptr<Frame> &pframe) override;

    private:
        void send_visualization();

        struct Pose {
            quatd rotation;
            vec3d translation;
            std::unordered_map<size_t, vec2d> observations;
        };

        mat3d m_K;

        std::vector<Pose> m_keyframes;
        std::vector<vec3d> m_landmarks;

        std::shared_ptr<Frame> m_last_keyframe;

        std::unique_ptr<FourPointPnPRANSAC> m_pnp;
        std::unique_ptr<Triangulator> m_triangulator;
    };

}