#include "logging/pose_logger.h"

bool PoseLogger::open(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) return false;
    file_.precision(9);  // ~nm/µs resolution, enough for pose + time
    t0_ = std::chrono::steady_clock::now();
    file_ << "t_s,source,x,y,z,qw,qx,qy,qz,frame,rs_ts_ms\n";
    file_.flush();
    return true;
}

double PoseLogger::nowSeconds() const {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - t0_).count();
}

void PoseLogger::log(const char* source,
                     double x, double y, double z,
                     double qw, double qx, double qy, double qz) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_.is_open()) return;
    // pose row: x..qz filled, frame/rs_ts_ms left empty.
    file_ << nowSeconds() << ',' << source << ','
          << x << ',' << y << ',' << z << ','
          << qw << ',' << qx << ',' << qy << ',' << qz << ",,\n";
    // Flush per row: logging rates are low (<~120 Hz) and the program is usually
    // stopped by a kill signal, so we favour not losing the tail over I/O cost.
    file_.flush();
}

void PoseLogger::logDepthFrame(unsigned long long frameNumber, double rsTimestampMs) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_.is_open()) return;
    // depth row: x..qz left empty, frame/rs_ts_ms filled.
    file_ << nowSeconds() << ",depth,,,,,,,," << frameNumber << ',' << rsTimestampMs << '\n';
    file_.flush();
}

void PoseLogger::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) file_.close();
}
