#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>

#include "depth_processing.h"
#include "cnn/cnn_encoder.h"
#ifdef USE_VO
#include "visual_odometry.h"
#endif

int main(int argc, char** argv) {
    // --record <path.bag>  : capture the raw camera stream to a file while running live
    // --replay <path.bag>  : run the whole pipeline (VO, policy input, display) off a
    //                        previously recorded file instead of a live camera
    std::string recordPath, replayPath;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--record" && i + 1 < argc) {
            recordPath = argv[++i];
        } else if (arg == "--replay" && i + 1 < argc) {
            replayPath = argv[++i];
        }
    }

    rs2::pipeline pipe;
    rs2::config cfg;
    if (!replayPath.empty()) {
        cfg.enable_device_from_file(replayPath);
    } else {
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        if (!recordPath.empty()) cfg.enable_record_to_file(recordPath);
    }
    rs2::pipeline_profile profile = pipe.start(cfg);

#ifndef NO_DISPLAY
    const bool displayEnabled = std::getenv("DISPLAY") != nullptr;
#endif

    float depthScale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    // cnn_preprocess_u16() (via cnn_encode_frame()) hardcodes a millimetre raw unit; the API
    // doesn't guarantee that, so fail loudly instead of silently feeding the policy garbage.
    if (std::abs(depthScale - 0.001f) > 1e-6f) {
        std::cerr << "Depth sensor scale is " << depthScale
                  << " m/unit, but the CNN encoder assumes exactly 0.001 (millimetres). Aborting.\n";
        return 1;
    }

#ifdef USE_VO
    // Visual odometry runs on color-aligned depth, kept separate from the
    // native-depth policy pipeline above so the RL input never depends on
    // whether/how VO is wired up.
    rs2::align alignToColor(RS2_STREAM_COLOR);
    rs2_intrinsics colorIntrin =
        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    const int VO_W = 320, VO_H = 240; // VO runs downsampled from the 640x480 capture
    const double voScaleX = static_cast<double>(VO_W) / colorIntrin.width;
    const double voScaleY = static_cast<double>(VO_H) / colorIntrin.height;
    cv::Mat colorK = (cv::Mat_<double>(3, 3) <<
        colorIntrin.fx * voScaleX, 0, colorIntrin.ppx * voScaleX,
        0, colorIntrin.fy * voScaleY, colorIntrin.ppy * voScaleY,
        0, 0, 1);
    // "RgbdOdometry" (photometric-only) is the fastest of the three variants
    // OpenCV offers; swap in "ICPOdometry" or "RgbdICPOdometry" for more
    // accuracy at higher CPU cost once you've measured headroom in the loop.
    VisualOdometry vo(colorK, "RgbdOdometry", /*minDepth=*/0.3f, /*maxDepth=*/4.0f);
#endif

    const int STAGE1_W = 64, STAGE1_H = 48;   // first downsample
#ifndef NO_DISPLAY
    const int POOL_SIZE = 4;                    // 4x4 max pool -> 16x12, for the display only
    const int DISPLAY_SCALE = 40;               // 16x12 -> 640x480 window
#endif

    // Fixed-rate control loop at 20 Hz. next_tick advances by a fixed period
    // each iteration (rather than "now + period"), so occasional overruns
    // don't accumulate drift in the schedule.
    using clock = std::chrono::steady_clock;
    const auto LOOP_PERIOD = std::chrono::duration<double>(1.0 / 20.0);
    auto next_tick = clock::now();

    // Sleeps until next_tick if there's time left, otherwise reports how far behind
    // schedule this iteration finished. context distinguishes *why* in the log line.
    auto waitForNextTick = [&](const char* context) {
        if (clock::now() < next_tick) {
            std::this_thread::sleep_until(next_tick);
        } else {
            std::cerr << "20 Hz loop overrun by "
                      << std::chrono::duration<double, std::milli>(clock::now() - next_tick).count()
                      << " ms" << context << "\n";
        }
    };

    while (true) {
        next_tick += std::chrono::duration_cast<clock::duration>(LOOP_PERIOD);

        // Drain the frame queue and keep only the newest frameset, so a
        // slower-than-camera loop never falls behind on stale frames.
        rs2::frameset frames;
        bool got_frame = false;
        rs2::frameset polled;
        while (pipe.poll_for_frames(&polled)) {
            frames = polled;
            got_frame = true;
        }
        if (!got_frame) {
            waitForNextTick(" (no new frame)");
            continue;
        }

        rs2::depth_frame depth = frames.get_depth_frame();
        if (!depth) continue;

#ifdef USE_VO
        // Color-aligned depth + grayscale, for VO only.
        rs2::frameset alignedFrames = alignToColor.process(frames);
        rs2::video_frame colorFrame = alignedFrames.get_color_frame();
        rs2::depth_frame depthAligned = alignedFrames.get_depth_frame();
        if (colorFrame && depthAligned) {
            cv::Mat colorMat(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3,
                              (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat grayFull;
            cv::cvtColor(colorMat, grayFull, cv::COLOR_BGR2GRAY);
            cv::Mat gray;
            cv::resize(grayFull, gray, cv::Size(VO_W, VO_H), 0, 0, cv::INTER_AREA);

            cv::Mat rawDepthAligned(cv::Size(depthAligned.get_width(), depthAligned.get_height()),
                                     CV_16UC1, (void*)depthAligned.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depthAlignedSmall;
            cv::resize(rawDepthAligned, depthAlignedSmall, cv::Size(VO_W, VO_H), 0, 0, cv::INTER_NEAREST);
            cv::Mat depthMeters;
            depthAlignedSmall.convertTo(depthMeters, CV_32FC1, depthScale);

            auto voStart = clock::now();
            bool voOk = vo.update(gray, depthMeters);
            double voMs = std::chrono::duration<double, std::milli>(clock::now() - voStart).count();

            if (voOk) {
                cv::Vec3d t = vo.translation();
                cv::Vec4d q = vo.quaternion(); // (w, x, y, z), FRD
                std::cerr << "VO pos: [" << t[0] << ", " << t[1] << ", " << t[2]
                          << "] quat: [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3]
                          << "] (" << voMs << " ms)\n";
            }
            // TODO: feed vo.pose() to whatever consumes odometry (state estimator, logging, etc.)
        }
#endif

        int w = depth.get_width();
        int h = depth.get_height();

        // Wrap raw RealSense depth buffer as an OpenCV Mat (no copy)
        cv::Mat rawDepth(cv::Size(w, h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        // Downsample to CNN_RAW_W x CNN_RAW_H (nearest-neighbor keeps real sample values)
        cv::Mat small;
        cv::resize(rawDepth, small, cv::Size(STAGE1_W, STAGE1_H), 0, 0, cv::INTER_NEAREST);

        // cnn_encode_frame() runs the training-matched normalize + 4x4 max-pool and the encoder
        // forward pass in one call, straight off the raw millimetre buffer (small is freshly
        // allocated, so it's contiguous). Built with CNN_INVALID_IS_FAR=1 (see Makefile): invalid/0
        // readings map to max range (far), not the near/blind-zone default.
        float features[CNN_FEATURE_DIM];
        cnn_encode_frame(small.ptr<uint16_t>(), features);
        // TODO: feed features to the FC (recurrent half, see ../fc/)

#ifndef NO_DISPLAY
        if (displayEnabled) {
            // 4x4 max pooling: 64x48 -> 16x12
            cv::Mat pooled = depthPool(small, POOL_SIZE, /*useMin=*/false);

            // Normalize for viewing (grayscale)
            double minVal, maxVal;
            cv::minMaxLoc(pooled, &minVal, &maxVal);
            cv::Mat normalized;
            pooled.convertTo(normalized, CV_8UC1, 255.0 / (maxVal > 0 ? maxVal : 1));

            // Upscale just for display (blocky, no smoothing)
            cv::Mat display;
            cv::resize(normalized, display, normalized.size() * DISPLAY_SCALE, 0, 0, cv::INTER_NEAREST);

            cv::imshow("Depth Stream (16x12 max-pooled)", display);
            if (cv::waitKey(1) == 27) break; // ESC to quit
        }
#endif

        waitForNextTick("");
    }

    return 0;
}
