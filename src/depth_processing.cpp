#include "depth_processing.h"

#include <limits>

cv::Mat depthPool(const cv::Mat& src, int poolSize, bool useMin) {
    int outW = src.cols / poolSize;
    int outH = src.rows / poolSize;
    cv::Mat dst(outH, outW, CV_16UC1);

    for (int y = 0; y < outH; y++) {
        for (int x = 0; x < outW; x++) {
            uint16_t best = useMin ? std::numeric_limits<uint16_t>::max() : 0;
            bool found = false;

            for (int dy = 0; dy < poolSize; dy++) {
                for (int dx = 0; dx < poolSize; dx++) {
                    uint16_t v = src.at<uint16_t>(y * poolSize + dy, x * poolSize + dx);
                    if (v == 0) continue; // skip invalid readings
                    if (useMin) { if (v < best) { best = v; found = true; } }
                    else        { if (v > best) { best = v; found = true; } }
                }
            }
            dst.at<uint16_t>(y, x) = found ? best : 0;
        }
    }
    return dst;
}
