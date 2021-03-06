#pragma once

#include <opencv2/opencv.hpp>
#include "Types.h"
#include "OcvImage.h"
#include "OcvImage_Impl.h"
#include "OcvOrbFeature.h"
#include "OcvOrbFeature_Impl.h"

namespace slam {

    struct OcvHelperFunctions {

        static mat3 K;
        static const Image *current_image;

        static void save_image(const std::string &filepath) {
            if (current_image) {
                save_image(current_image, filepath);
            }
        }

        static void save_image(const Image *image, const std::string &filepath) {
            const OcvImage *ocvimage = dynamic_cast<const OcvImage *>(image);
            if (ocvimage == nullptr || !ocvimage->valid()) {
                return;
            }
            cv::imwrite(filepath, ocvimage->m_pimpl->image);
        }

        static void show_image(int delay = 0) {
            if (current_image) {
                show_image(current_image, delay);
            }
        }

        static void show_image(const Image *image, int delay = 0) {
            const OcvImage *ocvimage = dynamic_cast<const OcvImage *>(image);
            if (ocvimage == nullptr || !ocvimage->valid()) {
                return;
            }
            cv::imshow("OcvHelperFunctions::show_image", ocvimage->m_pimpl->image);
            cv::waitKey(delay);
        }

        static void show_keypoints(const Feature *feature, int delay = 0) {
            if (current_image) {
                show_keypoints(current_image, feature, delay);
            }
        }

        static void show_keypoints(const Image *image, const Feature *feature, int delay = 0) {
            const OcvImage *ocvimage = dynamic_cast<const OcvImage *>(image);
            const OcvOrbFeature *ocvfeature = dynamic_cast<const OcvOrbFeature *>(feature);
            if (ocvimage == nullptr || ocvfeature == nullptr || !ocvimage->valid()) {
                return;
            }
            std::vector<cv::KeyPoint> cvkeypoints(ocvfeature->keypoints.size());
            for (size_t i = 0; i < ocvfeature->keypoints.size(); ++i) {
                cvkeypoints[i].pt.x = ocvfeature->keypoints[i][0] * K(0, 0) + K(0, 2);
                cvkeypoints[i].pt.y = ocvfeature->keypoints[i][1] * K(1, 1) + K(1, 2);
            }

            cv::Mat img;
            cv::drawKeypoints(ocvimage->m_pimpl->image, cvkeypoints, img, cv::Scalar(0, 0, 255));
            cv::imshow("OcvHelperFunctions::show_keypoints", img);
            cv::waitKey(delay);
        }

        static void show_match(const Image *image_source, const Feature *feature_source, const Image *image_target, const Feature *feature_target, const match_vector &matches, int delay = 0) {
            const OcvImage *ocvimage_source = dynamic_cast<const OcvImage *>(image_source);
            const OcvImage *ocvimage_target = dynamic_cast<const OcvImage *>(image_target);
            const OcvOrbFeature *ocvfeature_source = dynamic_cast<const OcvOrbFeature *>(feature_source);
            const OcvOrbFeature *ocvfeature_target = dynamic_cast<const OcvOrbFeature *>(feature_target);

            if (ocvimage_source == nullptr || ocvimage_target == nullptr || ocvfeature_source == nullptr || ocvfeature_target == nullptr || !ocvimage_source->valid() || !ocvimage_target->valid()) {
                return;
            }

            std::vector<cv::KeyPoint> cvkeypoints_source(ocvfeature_source->keypoints.size());
            std::vector<cv::KeyPoint> cvkeypoints_target(ocvfeature_target->keypoints.size());

            for (size_t i = 0; i < ocvfeature_source->keypoints.size(); ++i) {
                cvkeypoints_source[i].pt.x = ocvfeature_source->keypoints[i][0] * K(0, 0) + K(0, 2);
                cvkeypoints_source[i].pt.y = ocvfeature_source->keypoints[i][1] * K(1, 1) + K(1, 2);
            }
            for (size_t i = 0; i < ocvfeature_target->keypoints.size(); ++i) {
                cvkeypoints_target[i].pt.x = ocvfeature_target->keypoints[i][0] * K(0, 0) + K(0, 2);
                cvkeypoints_target[i].pt.y = ocvfeature_target->keypoints[i][1] * K(1, 1) + K(1, 2);
            }

            std::vector<cv::DMatch> cvmatches(matches.size());
            for (size_t i = 0; i < matches.size(); ++i) {
                cvmatches[i].queryIdx = (int)matches[i].first;
                cvmatches[i].trainIdx = (int)matches[i].second;
            }

            cv::Mat img;
            cv::drawMatches(ocvimage_source->m_pimpl->image, cvkeypoints_source, ocvimage_target->m_pimpl->image, cvkeypoints_target, cvmatches, img, cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 128));
            cv::imshow("OcvHelperFunctions::show_match", img);
            cv::waitKey(delay);
        }

        static void show_match_overlayed(const Feature *feature_source, const Feature *feature_target, const match_vector &matches, int delay = 0) {
            if (current_image) {
                show_match_overlayed(current_image, feature_source, feature_target, matches, delay);
            }
        }

        static void show_match_overlayed(const Image *image_source, const Feature *feature_source, const Feature *feature_target, const match_vector &matches, int delay = 0) {
            const OcvImage *ocvimage_source = dynamic_cast<const OcvImage *>(image_source);
            const OcvOrbFeature *ocvfeature_source = dynamic_cast<const OcvOrbFeature *>(feature_source);
            const OcvOrbFeature *ocvfeature_target = dynamic_cast<const OcvOrbFeature *>(feature_target);

            if (ocvimage_source == nullptr || ocvfeature_source == nullptr || ocvfeature_target == nullptr || !ocvimage_source->valid()) {
                return;
            }

            std::vector<cv::KeyPoint> cvkeypoints_source(ocvfeature_source->keypoints.size());
            std::vector<cv::KeyPoint> cvkeypoints_target(ocvfeature_target->keypoints.size());

            for (size_t i = 0; i < ocvfeature_source->keypoints.size(); ++i) {
                cvkeypoints_source[i].pt.x = ocvfeature_source->keypoints[i][0] * K(0, 0) + K(0, 2);
                cvkeypoints_source[i].pt.y = ocvfeature_source->keypoints[i][1] * K(1, 1) + K(1, 2);
            }
            for (size_t i = 0; i < ocvfeature_target->keypoints.size(); ++i) {
                cvkeypoints_target[i].pt.x = ocvfeature_target->keypoints[i][0] * K(0, 0) + K(0, 2);
                cvkeypoints_target[i].pt.y = ocvfeature_target->keypoints[i][1] * K(1, 1) + K(1, 2);
            }

            cv::Mat img;
            cv::drawKeypoints(ocvimage_source->m_pimpl->image, cvkeypoints_target, img, cv::Scalar(0, 0, 128));
            cv::drawKeypoints(img, cvkeypoints_source, img, cv::Scalar(0, 255, 255));
            for (auto &m : matches) {
                cv::line(img, cvkeypoints_source[m.first].pt, cvkeypoints_target[m.second].pt, cv::Scalar(0, 0, 255));
            }
            cv::imshow("OcvHelperFunctions::show_match_overlayed", img);
            cv::waitKey(delay);
        }
    };

}
