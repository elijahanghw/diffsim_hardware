#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

#include "depth_processing.h"
#include "visual_odometry.h"

int main() {
    const float CAM_MAX_RANGE = 3.0f; // meters, must match training

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    rs2::pipeline_profile profile = pipe.start(cfg);

    float depthScale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();

    // Visual odometry runs on color-aligned depth, kept separate from the
    // native-depth policy pipeline above so the RL input never depends on
    // whether/how VO is wired up.
    rs2::align alignToColor(RS2_STREAM_COLOR);
    rs2_intrinsics colorIntrin =
        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    cv::Mat colorK = (cv::Mat_<double>(3, 3) <<
        colorIntrin.fx, 0, colorIntrin.ppx,
        0, colorIntrin.fy, colorIntrin.ppy,
        0, 0, 1);
    // "RgbdOdometry" (photometric-only) is the fastest of the three variants
    // OpenCV offers; swap in "ICPOdometry" or "RgbdICPOdometry" for more
    // accuracy at higher CPU cost once you've measured headroom in the loop.
    VisualOdometry vo(colorK, "RgbdOdometry", /*minDepth=*/0.3f, /*maxDepth=*/4.0f);

    const int STAGE1_W = 64, STAGE1_H = 48;   // first downsample
    const int POOL_SIZE = 4;                   // 4x4 max pool -> 16x12
    const int DISPLAY_SCALE = 40;               // 16x12 -> 640x480 window

    // Fixed-rate control loop at 20 Hz. next_tick advances by a fixed period
    // each iteration (rather than "now + period"), so occasional overruns
    // don't accumulate drift in the schedule.
    using clock = std::chrono::steady_clock;
    const auto LOOP_PERIOD = std::chrono::duration<double>(1.0 / 20.0);
    auto next_tick = clock::now();

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
            if (clock::now() < next_tick) std::this_thread::sleep_until(next_tick);
            continue;
        }

        rs2::depth_frame depth = frames.get_depth_frame();
        if (!depth) continue;

        // Color-aligned depth + grayscale, for VO only.
        rs2::frameset alignedFrames = alignToColor.process(frames);
        rs2::video_frame colorFrame = alignedFrames.get_color_frame();
        rs2::depth_frame depthAligned = alignedFrames.get_depth_frame();
        if (colorFrame && depthAligned) {
            cv::Mat colorMat(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3,
                              (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat grayFull;
            cv::cvtColor(colorMat, grayFull, cv::COLOR_BGR2GRAY);

            cv::Mat rawDepthAligned(cv::Size(depthAligned.get_width(), depthAligned.get_height()),
                                     CV_16UC1, (void*)depthAligned.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depthFullMeters;
            rawDepthAligned.convertTo(depthFullMeters, CV_32FC1, depthScale);

            auto voStart = clock::now();
            bool voOk = vo.update(grayFull, depthFullMeters);
            double voMs = std::chrono::duration<double, std::milli>(clock::now() - voStart).count();

            if (voOk) {
                cv::Vec3d t = vo.translation();
                std::cerr << "VO pos: [" << t[0] << ", " << t[1] << ", " << t[2]
                          << "] (" << voMs << " ms)\n";
            }
            // TODO: feed vo.pose() to whatever consumes odometry (state estimator, logging, etc.)
        }

        int w = depth.get_width();
        int h = depth.get_height();

        // Wrap raw RealSense depth buffer as an OpenCV Mat (no copy)
        cv::Mat rawDepth(cv::Size(w, h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        // Downsample to 64x48 (nearest-neighbor keeps real sample values)
        cv::Mat small;
        cv::resize(rawDepth, small, cv::Size(STAGE1_W, STAGE1_H), 0, 0, cv::INTER_NEAREST);

        // Convert to meters and run the training-matched normalize + max-pool
        cv::Mat smallMeters;
        small.convertTo(smallMeters, CV_32FC1, depthScale);
        cv::Mat policyInput = processedDepth(smallMeters, POOL_SIZE, CAM_MAX_RANGE); // 12x16 float
        // TODO: feed policyInput to the policy network

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

        if (clock::now() < next_tick) {
            std::this_thread::sleep_until(next_tick);
        } else {
            std::cerr << "20 Hz loop overrun by "
                      << std::chrono::duration<double, std::milli>(clock::now() - next_tick).count()
                      << " ms\n";
        }
    }

    return 0;
}
