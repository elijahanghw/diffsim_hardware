#include "logging/pose_logger.h"

bool PoseLogger::open(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) return false;
    file_.precision(9);  // ~nm/µs resolution, enough for pose + time
    t0_ = std::chrono::steady_clock::now();
    file_ << "t_s,source,x,y,z,qw,qx,qy,qz\n";
    file_.flush();
    return true;
}

void PoseLogger::log(const char* source,
                     double x, double y, double z,
                     double qw, double qx, double qy, double qz) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_.is_open()) return;
    double t = std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0_).count();
    file_ << t << ',' << source << ','
          << x << ',' << y << ',' << z << ','
          << qw << ',' << qx << ',' << qy << ',' << qz << '\n';
    // Flush per row: logging rates are low (<~120 Hz) and the program is usually
    // stopped by a kill signal, so we favour not losing the tail over I/O cost.
    file_.flush();
}

void PoseLogger::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) file_.close();
}
