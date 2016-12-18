#pragma once

#include <utility>
#include <vector>
#include <Eigen/Eigen>

namespace slam {

    typedef float           realf;
    typedef Eigen::Vector2f vec2f;
    typedef Eigen::Vector3f vec3f;
    typedef Eigen::Vector4f vec4f;
    typedef Eigen::VectorXf vecxf;
    typedef Eigen::Matrix2f mat2f;
    typedef Eigen::Matrix3f mat3f;
    typedef Eigen::Matrix4f mat4f;
    typedef Eigen::MatrixXf matxf;

    typedef double          reald;
    typedef Eigen::Vector2d vec2d;
    typedef Eigen::Vector3d vec3d;
    typedef Eigen::Vector4d vec4d;
    typedef Eigen::VectorXd vecxd;
    typedef Eigen::Matrix2d mat2d;
    typedef Eigen::Matrix3d mat3d;
    typedef Eigen::Matrix4d mat4d;
    typedef Eigen::MatrixXd matxd;

    typedef realf real;
    typedef vec2f vec2;
    typedef vec3f vec3;
    typedef vec4f vec4;
    typedef vecxf vecx;
    typedef mat2f mat2;
    typedef mat3f mat3;
    typedef mat4f mat4;
    typedef matxf matx;

    typedef std::vector<std::pair<size_t, size_t>> match_vector;

}
