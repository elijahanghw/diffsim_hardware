#pragma once

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>

// Thread-safe CSV logger for time-stamped poses from several sources so they can
// be plotted and compared offline — here "mocap" (ground truth, from the relay
// thread) against "vo" (the visual-odometry estimate, from the camera loop).
//
// All sources share one monotonic time base (seconds since open()), so rows
// written from different threads line up on a common timeline. Frame
// conventions are the caller's and are only distinguished by the `source` label
// (mocap is NED; VO is in its own frame with an FRD body quaternion).
//
// CSV columns: t_s,source,x,y,z,qw,qx,qy,qz
class PoseLogger {
public:
    // Opens (truncating) the CSV at `path`, writes the header, and starts the
    // clock. Returns false if the file can't be opened.
    bool open(const std::string& path);
    bool isOpen() const { return file_.is_open(); }

    // Append one pose row. Thread-safe; safe to call before open() (no-op).
    void log(const char* source,
             double x, double y, double z,
             double qw, double qx, double qy, double qz);

    void close();

private:
    std::ofstream file_;
    std::mutex mutex_;
    std::chrono::steady_clock::time_point t0_;
};
