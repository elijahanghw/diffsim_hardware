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
#include "logging/pose_logger.h"
#ifdef USE_VO
#include "visual_odometry.h"
#endif
#ifdef USE_RELAY
#include "relay/fc_relay.h"
#endif

int main(int argc, char** argv) {
    // --record <path.bag>  : capture the raw camera stream to a file while running live
    // --replay <path.bag>  : run the whole pipeline (VO, policy input, display) off a
    //                        previously recorded file instead of a live camera
    // --fc-serial <dev> --fc-baud <rate> : bridge to the indiflight FC over
    //                        pi-protocol serial (relays optitrack/setpoint/
    //                        keyboard UDP and transmits the CNN features as
    //                        NN_INPUT_CHUNK). Omit --fc-serial to run without
    //                        the FC link. Baud defaults to 500000. Only active
    //                        in USE_RELAY builds.
    // --log <path.csv>     : log timestamped mocap (relay) and VO poses to a CSV
    //                        on one timeline, for offline accuracy plots.
    // --no-depth-filter    : disable the librealsense post-processing chain on the
    //                        CNN depth input (spatial/temporal/hole-filling). On by
    //                        default; it cleans the raw sensor depth so it better
    //                        matches the clean depth the policy trained on.
    std::string recordPath, replayPath, logPath;
    bool depthFilter = true;
#ifdef USE_RELAY
    std::string fcSerial;
    int fcBaud = 500000;
#endif
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--record" && i + 1 < argc) {
            recordPath = argv[++i];
        } else if (arg == "--replay" && i + 1 < argc) {
            replayPath = argv[++i];
        } else if (arg == "--log" && i + 1 < argc) {
            logPath = argv[++i];
        } else if (arg == "--no-depth-filter") {
            depthFilter = false;
        }
#ifdef USE_RELAY
        else if (arg == "--fc-serial" && i + 1 < argc) {
            fcSerial = argv[++i];
        } else if (arg == "--fc-baud" && i + 1 < argc) {
            fcBaud = std::atoi(argv[++i]);
        }
#endif
    }

    // Pose logger (mocap vs VO). Enabled by --log; a no-op otherwise.
    PoseLogger poseLogger;
    bool logging = false;
    if (!logPath.empty()) {
        logging = poseLogger.open(logPath);
        if (!logging) {
            std::cerr << "Failed to open pose log file " << logPath << "\n";
        } else {
            std::cerr << "Logging mocap + VO poses to " << logPath << "\n";
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

#ifdef USE_RELAY
    // FC serial bridge. Owns the serial port on its own thread; the loop below
    // only hands it each new CNN feature vector via publishFeatures().
    FcRelay relay;
    bool relayActive = false;
    if (!fcSerial.empty()) {
        if (logging) relay.setMocapLogger(&poseLogger);
        relayActive = relay.start(fcSerial, fcBaud);
        if (!relayActive) {
            std::cerr << "FcRelay: failed to start on " << fcSerial << " @ " << fcBaud
                      << " baud; continuing without the FC link.\n";
        }
    } else {
        std::cerr << "No --fc-serial given; running without the FC link.\n";
    }
#endif

    const int STAGE1_W = 64, STAGE1_H = 48;   // first downsample
#ifndef NO_DISPLAY
    const int DISPLAY_SCALE = 40;               // 16x12 -> 640x480 window
#endif

    // librealsense post-processing chain for the CNN depth input, to clean the
    // raw sensor depth so it resembles the dense, noise-free depth the policy was
    // trained on (see README "Depth filtering"). Recommended order: convert to
    // disparity, spatial + temporal smoothing there, back to depth, then fill
    // holes. Stateful (temporal keeps history), so these persist across frames.
    // The VO path is left on unfiltered depth. Disable with --no-depth-filter.
    rs2::disparity_transform depth2disparity(true);
    rs2::disparity_transform disparity2depth(false);
    rs2::spatial_filter spatialFilter;
    rs2::temporal_filter temporalFilter;
    rs2::hole_filling_filter holeFilter;
    // Fill holes from the nearest surrounding depth: for obstacle avoidance it is
    // safer for a dropout next to an obstacle to read near than to read through it.
    holeFilter.set_option(RS2_OPTION_HOLES_FILL, 2.0f);  // 2 = nearest_from_around
    std::cerr << "Depth filtering: " << (depthFilter ? "on" : "off (--no-depth-filter)") << "\n";

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

        // Mark this frame on the log timeline so a --record .bag frame can be
        // matched (by number / hardware timestamp) to the nearest logged pose.
        if (logging) poseLogger.logDepthFrame(depth.get_frame_number(), depth.get_timestamp());

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

            bool voOk = vo.update(gray, depthMeters);
            if (voOk && logging) {
                cv::Vec3d t = vo.translation();
                cv::Vec4d q = vo.quaternion();  // (w, x, y, z), FRD
                poseLogger.log("vo", t[0], t[1], t[2], q[0], q[1], q[2], q[3]);
            }
            // TODO: feed vo.pose() to whatever consumes odometry (state estimator, logging, etc.)
        }
#endif

        // Clean the sensor depth (spatial/temporal/hole-filling) so it better
        // matches the dense, noise-free depth the policy trained on. The chain is
        // stateful across frames; VO above deliberately uses the unfiltered depth.
        rs2::depth_frame cnnDepth = depth;
        if (depthFilter) {
            rs2::frame f = depth;
            f = depth2disparity.process(f);
            f = spatialFilter.process(f);
            f = temporalFilter.process(f);
            f = disparity2depth.process(f);
            f = holeFilter.process(f);
            cnnDepth = f.as<rs2::depth_frame>();
        }

        int w = cnnDepth.get_width();
        int h = cnnDepth.get_height();

        // Wrap the (filtered) RealSense depth buffer as an OpenCV Mat (no copy)
        cv::Mat rawDepth(cv::Size(w, h), CV_16UC1, (void*)cnnDepth.get_data(), cv::Mat::AUTO_STEP);

        // Downsample to CNN_RAW_W x CNN_RAW_H by min-pooling the nearest valid
        // (non-zero) depth in each block. Unlike INTER_NEAREST (one sample per
        // block, which could land on a hole or skip a thin/near object), this
        // keeps the closest obstacle in every cell and ignores dropouts -- what
        // obstacle avoidance needs. 640x480 -> 64x48 is an exact 10x pooling.
        const int poolW = w / STAGE1_W;   // 640/64 = 10; 480/48 = 10 (square pooling)
        if (h / STAGE1_H != poolW) {
            std::cerr << "Unexpected depth resolution " << w << "x" << h << "; expected a multiple of "
                      << STAGE1_W << "x" << STAGE1_H << "\n";
        }
        cv::Mat small = depthPool(rawDepth, poolW, /*useMin=*/true);

        // Split of cnn_encode_frame() into its two steps so depth_in — the exact
        // normalized tensor the encoder consumes — is available for the preview
        // below. cnn_preprocess_u16() runs the training-matched normalize + 4x4
        // max-pool straight off the raw millimetre buffer (small is freshly
        // allocated, so it's contiguous). Built with CNN_INVALID_IS_FAR=1 (see
        // Makefile): invalid/0 readings map to max range (far), not near/blind.
        float depth_in[CNN_IN_H * CNN_IN_W];
        float features[CNN_FEATURE_DIM];
        cnn_preprocess_u16(small.ptr<uint16_t>(), depth_in);
        cnn_forward(depth_in, features);
#ifdef USE_RELAY
        // Hand the latest features to the relay thread, which transmits them to
        // the FC as NN_INPUT_CHUNK. The FC runs the recurrent half (see ../fc/).
        if (relayActive) relay.publishFeatures(features, CNN_FEATURE_DIM);
#endif

#ifndef NO_DISPLAY
        if (displayEnabled) {
            // Show exactly what the CNN sees: the CNN_IN_H x CNN_IN_W normalized
            // input (depth_in), post-inversion. Map the fixed value range to
            // grayscale so brightness is absolute across frames: the closest
            // surface (largest value) is white, the farthest (smallest) is black.
            constexpr float vNear = CNN_NORM_NUM / CNN_NORM_MIN  - CNN_NORM_OFF; // nearest
            constexpr float vFar  = CNN_NORM_NUM / CNN_MAX_RANGE - CNN_NORM_OFF; // farthest
            cv::Mat cnnView(CNN_IN_H, CNN_IN_W, CV_8UC1);
            for (int r = 0; r < CNN_IN_H; ++r) {
                for (int c = 0; c < CNN_IN_W; ++c) {
                    float s = 255.0f * (depth_in[r * CNN_IN_W + c] - vFar) / (vNear - vFar);
                    if (s < 0.0f) s = 0.0f;
                    if (s > 255.0f) s = 255.0f;
                    cnnView.at<uint8_t>(r, c) = static_cast<uint8_t>(s);
                }
            }

            // Upscale just for display (blocky, no smoothing)
            cv::Mat display;
            cv::resize(cnnView, display, cnnView.size() * DISPLAY_SCALE, 0, 0, cv::INTER_NEAREST);

            cv::imshow("CNN input (16x12, bright = near)", display);
            if (cv::waitKey(1) == 27) break; // ESC to quit
        }
#endif

        waitForNextTick("");
    }

#ifdef USE_RELAY
    relay.stop();
#endif
    poseLogger.close();
    return 0;
}
