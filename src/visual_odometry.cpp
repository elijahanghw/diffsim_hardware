#include "visual_odometry.h"

namespace {

cv::Ptr<cv::rgbd::Odometry> createOdometry(const std::string& type, const cv::Mat& K,
                                            float minDepth, float maxDepth) {
    if (type == "ICPOdometry") {
        return cv::rgbd::ICPOdometry::create(K, minDepth, maxDepth);
    }
    if (type == "RgbdICPOdometry") {
        return cv::rgbd::RgbdICPOdometry::create(K, minDepth, maxDepth);
    }
    return cv::rgbd::RgbdOdometry::create(K, minDepth, maxDepth);
}

} // namespace

VisualOdometry::VisualOdometry(const cv::Mat& cameraMatrix, const std::string& odometryType,
                                float minDepth, float maxDepth)
    : odometry_(createOdometry(odometryType, cameraMatrix, minDepth, maxDepth)),
      pose_(cv::Mat::eye(4, 4, CV_64F)) {}

bool VisualOdometry::update(const cv::Mat& grayImage, const cv::Mat& depthMeters) {
    if (!hasPrev_) {
        prevGray_ = grayImage.clone();
        prevDepth_ = depthMeters.clone();
        hasPrev_ = true;
        return false;
    }

    // src = current frame, dst = previous frame. OpenCV's documented
    // convention is dst_p = Rt * src_p, so Rt maps points from the CURRENT
    // camera frame into the PREVIOUS camera frame -- i.e. the incremental
    // motion to left-multiply onto the accumulated camera-to-world pose.
    cv::Mat Rt;
    bool ok = odometry_->compute(grayImage, depthMeters, cv::Mat(),
                                  prevGray_, prevDepth_, cv::Mat(), Rt);

    prevGray_ = grayImage.clone();
    prevDepth_ = depthMeters.clone();

    if (!ok) return false;

    pose_ = pose_ * Rt;
    return true;
}

cv::Vec3d VisualOdometry::translation() const {
    return cv::Vec3d(pose_.at<double>(0, 3), pose_.at<double>(1, 3), pose_.at<double>(2, 3));
}

void VisualOdometry::reset() {
    pose_ = cv::Mat::eye(4, 4, CV_64F);
    prevGray_.release();
    prevDepth_.release();
    hasPrev_ = false;
}
