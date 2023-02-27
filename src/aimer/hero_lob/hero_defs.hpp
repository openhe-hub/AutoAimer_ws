//
// Created by LingMisaki on 2022/12/7.
// 本文件是英雄吊射识别器和预测器的沟通桥梁

#ifndef TOS_HERO_DEFS_HPP
#define TOS_HERO_DEFS_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace hero {

struct GreenLightDetectionResult {
  cv::Mat img;
  Eigen::Quaternionf q;
  double timestamp;
  cv::Point2f pts[4];
  bool found;
};
} // namespace hero
#endif