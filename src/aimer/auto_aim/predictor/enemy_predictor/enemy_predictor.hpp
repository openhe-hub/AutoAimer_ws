//
// by fjj, 2022-04-25

#ifndef AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_ENEMY_PREDICTOR_HPP
#define AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_ENEMY_PREDICTOR_HPP

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <fmt/color.h>
#include <fmt/format.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "../../base/defs.hpp"
#include "../aim/aim_corrector.hpp"
#include "../enemy/armor_identifier.hpp"
#include "../enemy/enemy_state.hpp"
#include "../motion/enemy_model.hpp"
#include "../../../base/armor_defs.hpp"
#include "../../../base/debug/debug.hpp"
#include "../../../base/math/math.hpp"
#include "../../../base/robot/coord_converter.hpp"
#include "core_io/robot.hpp"

// 不在自瞄也要进行所有 kalman，只是不发送电控指令，且选敌逻辑有区别

// 系统信息，本质不同的坐标变换或者需要系统参数的变换的都需我的实例
// 目前，涉及打击决策或者时间线程管理的类都需要获得它的指针
namespace aimer {

/**
 * @brief 按照一定延迟规则捕获目标
 *
 */
class TargetCatcher {
 public:
  TargetCatcher(aimer::CoordConverter* const converter,
                aimer::EnemyState* const states);
  void catch_target(const int& target_number);
  int get_target() const;

 private:
  // add clock!
  aimer::CoordConverter* const converter;
  aimer::EnemyState* const states;
  int target_number = -1;
  double target_caught_t = 0.;  // not "first caught t"
};
// get_target: 根据最近 caught 时间决定目标是否丢失 (-1)
// armor_data 的存在: 根据数据的存在决定未丢失的目标是否打击
// 当 empty 时，目标并不会丢失。接下来仍然优先打击目标

class DetectedFixer {
 public:
  [[deprecated]] aimer::DetectedArmor fixed(
      const aimer::DetectedArmor& detected) {
    aimer::DetectedArmor res = detected;
    if (res.number == 6 || res.number == 7 || res.number == 8)
      res.number = 6;
    return res;
  }

 private:
};

/**
 * @brief
 *
 */
class EnemyPredictor {
 public:
  EnemyPredictor();
  cv::Mat draw_aim(const cv::Mat& img);
  cv::Mat draw_map();
  ::RobotCmd predict(const aimer::DetectionResult& data);  // 主函数

 private:
  /** @brief 更新数据库 */
  void update_database(const aimer::DetectionResult& data);
  /** @brief 检查模型类型 */
  void check_models();
  /** @brief 获取全部经过筛选的装甲板数据 */
  void get_all_armors();
  /** @brief 获取目标 */
  void get_target();
  /** @brief 更新模型运动状态 */
  void update_models();
  /** @brief 从图像中估计枪口指向的理想实际误差 */
  void update_aim_error();
  /** @brief 计算瞄准信息 */
  aimer::AimInfo get_aim();
  /** @brief 获取控制信息 */
  ::RobotCmd get_cmd_by_aim(const aimer::AimInfo& aim);

  //   成员访问它相当于我把它传出去
  aimer::CoordConverter converter;
  aimer::EnemyState enemy_states[aimer::MAX_ENEMY_NUMBER + 1];
  aimer::DetectedFixer detected_fixer;
  aimer::LightManager light;
  // aim 总是瞄准点的意思，target 是 target_number 的简称
  // target 指的是要打击的敌人编号
  aimer::TargetCatcher target_catcher;
  std::vector<aimer::ArmorData> all_armors;
  // 可更新或空置的 Model
  std::unique_ptr<aimer::EnemyModelInterface>
      enemy_models[aimer::MAX_ENEMY_NUMBER + 1];
  aimer::EnemyModelFactory model_factory;
  // aimer::AimHistory aim_history;
  int aim_id_cnt = 0;
  aimer::aim::AimCorrector aim_corrector;

  // for debug
  aimer::debug::CmdDebugger cmd_debugger;
  aimer::debug::Stm32Shoot stm32_shoot;
  aimer::debug::PeriodicRecorder<double, double> processing_time_recorder{
      ([](const double& t, const double& t1) -> bool { return t < t1; }),
      -86400.};
  aimer::debug::PeriodicAverage<double, double> desired_yaw_average{200u,
                                                                    -86400.};
  std::set<int> debug_update_list;
  aimer::AimInfo debug_aim = aimer::AimInfo::idle();
  ::RobotCmd debug_cmd;
  double cmd_last_shoot_t = 0.;
};
}  // namespace aimer

#endif /* AIMER_AUTO_AIM_PREDICTOR_ENEMY_PREDICTOR_ENEMY_PREDICTOR_HPP */
