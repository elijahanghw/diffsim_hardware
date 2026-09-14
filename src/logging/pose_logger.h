#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

// Asynchronous, thread-safe CSV logger that puts several kinds of time-stamped
// rows on one shared monotonic timeline (seconds since open()), so they can be
// plotted, compared and synced offline:
//   - "mocap" pose (ground truth, from the relay thread),
//   - "vo" pose (the visual-odometry estimate, from the camera loop),
//   - "depth" frame markers (one per processed depth frame), which tie each
//     recorded RealSense frame (by frame number / hardware timestamp, so it can
//     be matched to a --record .bag) to this timeline.
//
// IMPORTANT: log()/logDepthFrame() do NO disk I/O. They format the row and hand
// it to a background writer thread through an in-memory queue, so the real-time
// callers (the FC relay thread, the camera loop) never block on file writes —
// disk-latency spikes on slow storage would otherwise stall the serial relay and
// make the FC miss mocap. Only the writer thread touches the file.
//
// CSV columns: t_s,source,x,y,z,qw,qx,qy,qz,frame,rs_ts_ms
//   pose rows fill x..qz and leave frame/rs_ts_ms empty; depth rows do the
//   reverse.
class PoseLogger {
public:
    PoseLogger() = default;
    ~PoseLogger();

    PoseLogger(const PoseLogger&) = delete;
    PoseLogger& operator=(const PoseLogger&) = delete;

    // Opens (truncating) the CSV at `path`, writes the header, starts the clock,
    // and spawns the writer thread. Returns false if the file can't be opened.
    bool open(const std::string& path);
    bool isOpen() const { return open_.load(); }

    // Append one pose row. Thread-safe, non-blocking; safe before open() (no-op).
    void log(const char* source,
             double x, double y, double z,
             double qw, double qx, double qy, double qz);

    // Append one depth-frame marker row tying a recorded frame to this timeline.
    // Thread-safe, non-blocking; safe before open() (no-op).
    void logDepthFrame(unsigned long long frameNumber, double rsTimestampMs);

    // Flushes the queue, stops the writer thread, closes the file. Idempotent;
    // also called by the destructor.
    void close();

private:
    double nowSeconds() const;   // seconds since t0_
    void enqueue(std::string line);
    void writerLoop();

    std::ofstream file_;
    std::atomic<bool> open_{false};
    std::chrono::steady_clock::time_point t0_;

    std::thread writer_;
    std::mutex qMutex_;
    std::condition_variable qCv_;
    std::deque<std::string> queue_;
    bool stop_ = false;
};
