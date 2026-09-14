#include "logging/pose_logger.h"

#include <cstdio>

bool PoseLogger::open(const std::string& path) {
    if (open_.load()) return true;
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) return false;
    file_ << "t_s,source,x,y,z,qw,qx,qy,qz,frame,rs_ts_ms\n";
    file_.flush();
    t0_ = std::chrono::steady_clock::now();
    stop_ = false;
    open_.store(true);
    writer_ = std::thread(&PoseLogger::writerLoop, this);
    return true;
}

double PoseLogger::nowSeconds() const {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - t0_).count();
}

void PoseLogger::enqueue(std::string line) {
    {
        std::lock_guard<std::mutex> lock(qMutex_);
        queue_.push_back(std::move(line));
    }
    qCv_.notify_one();
}

void PoseLogger::log(const char* source,
                     double x, double y, double z,
                     double qw, double qx, double qy, double qz) {
    if (!open_.load()) return;
    // pose row: x..qz filled, frame/rs_ts_ms left empty. %.9g ~ 9 significant
    // digits, matching the old stream precision. Formatting only — no disk I/O.
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "%.9g,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,,\n",
                  nowSeconds(), source, x, y, z, qw, qx, qy, qz);
    enqueue(std::string(buf));
}

void PoseLogger::logDepthFrame(unsigned long long frameNumber, double rsTimestampMs) {
    if (!open_.load()) return;
    // depth row: x..qz left empty, frame/rs_ts_ms filled.
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%.9g,depth,,,,,,,,%llu,%.9g\n",
                  nowSeconds(), frameNumber, rsTimestampMs);
    enqueue(std::string(buf));
}

void PoseLogger::writerLoop() {
    std::deque<std::string> batch;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(qMutex_);
            qCv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
            if (queue_.empty() && stop_) break;
            batch.swap(queue_);  // take the backlog, release the lock before I/O
        }
        for (const auto& line : batch) file_ << line;
        file_.flush();
        batch.clear();
    }
    file_.flush();
}

void PoseLogger::close() {
    if (!open_.load()) return;
    {
        std::lock_guard<std::mutex> lock(qMutex_);
        stop_ = true;
    }
    qCv_.notify_one();
    if (writer_.joinable()) writer_.join();
    file_.close();
    open_.store(false);
}

PoseLogger::~PoseLogger() {
    close();
}
