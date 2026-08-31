#pragma once

#include <opencv2/core.hpp>
#include <opencv2/rgbd.hpp>
#include <string>

// Frame-to-frame RGB-D visual odometry, built on OpenCV's rgbd module
// (cv::rgbd::RgbdOdometry / ICPOdometry / RgbdICPOdometry).
//
// grayImage/depthMeters passed to update() must be pixel-aligned to the SAME
// camera intrinsics passed to the constructor (e.g. depth aligned to the
// color sensor via rs2::align, using the color stream's intrinsics as K).
//
// This is plain frame-to-frame odometry: no loop closure, so pose drifts
// over time like any dead-reckoning estimate.
class VisualOdometry {
public:
    // odometryType: "RgbdOdometry" (photometric only, fastest),
    // "ICPOdometry" (geometric only), or "RgbdICPOdometry" (both combined,
    // most accurate but slowest). cameraMatrix is the 3x3 intrinsics
    // (fx,0,cx; 0,fy,cy; 0,0,1), CV_64F.
    VisualOdometry(const cv::Mat& cameraMatrix,
                    const std::string& odometryType = "RgbdOdometry",
                    float minDepth = 0.3f, float maxDepth = 4.0f);

    // Feeds one new frame. grayImage: CV_8UC1. depthMeters: CV_32FC1, meters.
    // Returns false on the very first call (nothing to compare against yet)
    // or if this frame's relative transform could not be computed
    // (e.g. too few correspondences) -- pose() is left unchanged either way.
    bool update(const cv::Mat& grayImage, const cv::Mat& depthMeters);

    // Accumulated camera-to-world pose (4x4, CV_64F); world = the pose at
    // the first successful update().
    const cv::Mat& pose() const { return pose_; }

    // Convenience accessor for the translation component of pose().
    cv::Vec3d translation() const;

    void reset();

private:
    cv::Ptr<cv::rgbd::Odometry> odometry_;
    cv::Mat prevGray_, prevDepth_;
    cv::Mat pose_;
    bool hasPrev_ = false;
};
