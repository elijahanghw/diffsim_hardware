#pragma once

#include <opencv2/core.hpp>

// poolSize x poolSize pooling on raw uint16 depth data.
// useMin=true  -> nearest object per block (good for obstacle avoidance)
// useMin=false -> farthest object per block (max pooling)
// Zero values (invalid readings) are ignored unless a block is all zero.
cv::Mat depthPool(const cv::Mat& src, int poolSize, bool useMin);

// Matches the training-time depth processing:
//   normd = 3.0 / clip(raw_m, 0.3, camMaxRange) - 0.6
//   4x4 max-pool over normd (not over raw depth)
// Invalid readings (raw == 0, e.g. out of range / low reflectivity) are
// treated as camMaxRange so they end up as the *lowest* normalized value
// instead of masquerading as the nearest possible obstacle.
cv::Mat processedDepth(const cv::Mat& depthMeters, int poolSize, float camMaxRange);
