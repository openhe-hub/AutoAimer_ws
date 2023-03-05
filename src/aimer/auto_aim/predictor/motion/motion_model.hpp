/*敌方正常运动预测模块*/

#ifndef AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_MODEL_HPP
#define AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_MODEL_HPP

#include <iostream>  // debug 用
#include <opencv2/opencv.hpp>
#include <vector>

#include <ceres/ceres.h>
#include <fmt/format.h>

#include "../../base/defs.hpp"
#include "../enemy/enemy_state.hpp"
#include "../../../base/armor_defs.hpp"
#include "../../../base/math/filter/filter.hpp"
#include "../../../base/math/math.hpp"
#include "../../../base/robot/coord_converter.hpp"

namespace aimer {
// 滤波器线程
struct FilterThread {
 public:
  // 建立也是更新
  FilterThread(aimer::CoordConverter* const converter,
               const aimer::ArmorData& armor,
               const double& credit_time);

  void update(const aimer::ArmorData& armor, const double& t);

  bool credit() const;

  // 均采用速度模型
  // 注意电控 PID 是双环，无法准确进行二阶打击，视觉也不会发送加速度
  // 咱也不懂这个 R 参数应该怎么调的 (R 是转移方差)
  aimer::PositionEkf filter;
  aimer::ArmorData armor;

 private:
  aimer::CreditClock credit_clock;
};

// 普通运动模型，不断为可见装甲板创建线程
class MotionModel {
 public:
  //  最不可缺少参数的放在参数列表前面
  MotionModel(aimer::CoordConverter* const converter,
              aimer::EnemyState* const state,
              const double& credit_time);
  ~MotionModel() = default;
  void update();
  void draw_aim(cv::Mat& img) const;

  struct GetAimOption {
    explicit GetAimOption(const bool& passive) : passive(passive) {}
    const bool& passive;
  };

  aimer::AimInfo get_aim(const MotionModel::GetAimOption& option) const;

 private:
  aimer::AimInfo get_positive_aim() const;
  aimer::AimInfo get_passive_aim() const;
  aimer::CoordConverter* const converter;
  aimer::EnemyState* const state;
  const double credit_time;
  // 要用 armor 的 id 去寻找对应的 filter
  std::map<int, aimer::FilterThread> filters;
};
}  // namespace aimer

#endif /* AIMER_AUTO_AIM_PREDICTOR_MOTION_MOTION_MODEL_HPP */
