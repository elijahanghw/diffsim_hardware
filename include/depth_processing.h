#pragma once

#include <opencv2/core.hpp>

// poolSize x poolSize pooling on raw uint16 depth data.
// useMin=true  -> nearest object per block (good for obstacle avoidance)
// useMin=false -> farthest object per block (max pooling)
// Zero values (invalid readings) are ignored unless a block is all zero.
cv::Mat depthPool(const cv::Mat& src, int poolSize, bool useMin);
