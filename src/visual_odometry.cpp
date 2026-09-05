#include "visual_odometry.h"
#include <opencv2/core/quaternion.hpp>

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

// Camera-optical (X right, Y down, Z forward) -> FRD (X forward, Y right,
// Z down). Both are right-handed, so this is a pure axis permutation with
// no reflection: v_frd = camToFrd * v_cam.
const cv::Mat& camToFrd() {
    static const cv::Mat P = (cv::Mat_<double>(3, 3) <<
        0, 0, 1,
        1, 0, 0,
        0, 1, 0);
    return P;
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

cv::Mat VisualOdometry::pose() const {
    const cv::Mat& P = camToFrd();
    cv::Mat R = pose_(cv::Rect(0, 0, 3, 3));
    cv::Mat t = pose_(cv::Rect(3, 0, 1, 3));

    cv::Mat frd = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat(P * R * P.t()).copyTo(frd(cv::Rect(0, 0, 3, 3)));
    cv::Mat(P * t).copyTo(frd(cv::Rect(3, 0, 1, 3)));
    return frd;
}

cv::Vec3d VisualOdometry::translation() const {
    cv::Mat t = camToFrd() * pose_(cv::Rect(3, 0, 1, 3));
    return cv::Vec3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));
}

cv::Vec4d VisualOdometry::quaternion() const {
    const cv::Mat& P = camToFrd();
    cv::Mat R = P * pose_(cv::Rect(0, 0, 3, 3)) * P.t();
    cv::Quatd q = cv::Quatd::createFromRotMat(R);
    return cv::Vec4d(q.w, q.x, q.y, q.z);
}

void VisualOdometry::reset() {
    pose_ = cv::Mat::eye(4, 4, CV_64F);
    prevGray_.release();
    prevDepth_.release();
    hasPrev_ = false;
}
