#pragma once

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>

// Thread-safe CSV logger that puts several kinds of time-stamped rows on one
// shared monotonic timeline (seconds since open()), so they can be plotted,
// compared and synced offline:
//   - "mocap" pose (ground truth, from the relay thread),
//   - "vo" pose (the visual-odometry estimate, from the camera loop),
//   - "depth" frame markers (one per processed depth frame), which tie each
//     recorded RealSense frame (by frame number / hardware timestamp, so it can
//     be matched to a --record .bag) to this timeline.
//
// Because every row shares the one t_s clock, a .bag depth frame maps to a t_s,
// and from there to the nearest mocap/vo pose — enough to build overlay videos
// or aligned plots. Frame conventions are the caller's, distinguished only by
// the `source` label (mocap is NED; VO is in its own frame with an FRD body
// quaternion).
//
// CSV columns: t_s,source,x,y,z,qw,qx,qy,qz,frame,rs_ts_ms
//   pose rows fill x..qz and leave frame/rs_ts_ms empty; depth rows do the
//   reverse.
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

    // Append one depth-frame marker row tying a recorded frame to this timeline.
    // Thread-safe; safe to call before open() (no-op).
    void logDepthFrame(unsigned long long frameNumber, double rsTimestampMs);

    void close();

private:
    // Seconds since t0_. Caller must hold mutex_.
    double nowSeconds() const;

    std::ofstream file_;
    std::mutex mutex_;
    std::chrono::steady_clock::time_point t0_;
};
