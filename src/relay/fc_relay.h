#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

// FcRelay — on-board bridge from this program to the indiflight flight
// controller over the pi-protocol serial link.
//
// It is the companion-computer counterpart of the standalone relay
// (relay_diffsim): it owns the FC serial port and runs a background thread that
//   - reads EKF_INPUTS from the FC (for the time_us timestamp), and relays
//     UDP inputs to the FC: optitrack pose -> FAKE_GPS + EXTERNAL_POSE
//     (port 5005), POS_SETPOINT (5006), KEYBOARD (5007);
//   - transmits the CNN feature vector as NN_INPUT_CHUNK messages.
//
// Unlike the standalone relay, it does NOT listen on the HITL feature port
// (5010): here the features come straight from the on-board CNN via
// publishFeatures(). The standalone relay remains the tool for HITL.
//
// Threading: the FC serial port has a single writer (the relay thread). The
// camera/CNN loop only calls publishFeatures(), which hands the latest vector
// across under a mutex; the relay thread transmits it on its own schedule.
class FcRelay {
public:
    static constexpr int kFeatureDim = 64;

    FcRelay() = default;
    ~FcRelay();

    FcRelay(const FcRelay&) = delete;
    FcRelay& operator=(const FcRelay&) = delete;

    // Opens the serial port at baudRateHz (e.g. 921600), binds the UDP input
    // ports, and starts the relay thread. Returns false (with a message on
    // stderr) if the serial port can't be opened or the baud rate isn't
    // supported; the UDP binds abort the process on failure, matching the
    // standalone relay.
    bool start(const std::string& serialPort, int baudRateHz);

    // Hand the latest CNN feature vector to the relay thread. Copies n floats
    // (must equal kFeatureDim). Cheap and lock-guarded; safe to call every
    // frame from the camera loop.
    void publishFeatures(const float* features, std::size_t n);

    // Stops the relay thread and closes all fds. Idempotent; also called by the
    // destructor.
    void stop();

private:
    void run();
    void sendNnFeatures(const std::array<float, kFeatureDim>& vals);

    std::thread thread_;
    std::atomic<bool> running_{false};

    int serialFd_ = -1;
    int optitrackFd_ = -1;
    int setpointFd_ = -1;
    int keyboardFd_ = -1;

    // Latest features published by the camera thread. seq_ advances on every
    // publish; the relay thread transmits only when it sees a new seq_.
    std::mutex featuresMutex_;
    std::array<float, kFeatureDim> features_{};
    std::uint64_t featuresSeq_ = 0;
};
