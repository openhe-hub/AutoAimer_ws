#include "parameter.hpp"
#include <cstddef>
#include "./base/debug/print.hpp"

namespace aimer {
namespace param {

namespace base = ::base;

std::shared_ptr<Integer> create_int_obj(const std::string& str) {
  auto res = umt::ObjManager<Integer>::create("AIMER_" + str);
  if (res == nullptr) {
    base::print(
        base::PrintMode::Warning, "param",
        "create_int_obj 中参数 \"{}\" 已经被创建, 但你又尝试创建并被拒绝.\n",
        str);
  }
  return res;
}

std::shared_ptr<Integer> find_int_obj(const std::string& str) {
  auto res = umt::ObjManager<Integer>::find("AIMER_" + str);
  if (res == nullptr) {
    base::print(base::PrintMode::Warning, "param",
                "find_int_obj 找不到传入的键 \"{}\", 并将返回 nullptr.\n", str);
  }
  return res;
}

int find_int_param(const std::string& str) {
  return param::find_int_obj(str)->value;
}

std::shared_ptr<Double> create_double_obj(const std::string& str) {
  auto res = umt::ObjManager<Double>::create("AIMER_" + str);
  if (res == nullptr) {
    base::print(
        base::PrintMode::Warning, "param",
        "create_double_obj 中参数 \"{}\" 已经被创建, 但你又尝试创建并被拒绝.\n",
        str);
  }
  return res;
}

std::shared_ptr<Double> find_double_obj(const std::string& str) {
  auto res = umt::ObjManager<Double>::find("AIMER_" + str);
  if (res == nullptr) {
    base::print(base::PrintMode::Warning, "param",
                "find_double_obj 找不到传入的键 \"{}\", 并将返回 nullptr.\n",
                str);
  }
  return res;
}

double find_double_param(const std::string& str) {
  return param::find_double_obj(str)->value;
}

void PredictorParameter::load() {
  using namespace std::chrono_literals;
  // 对局信息
  this->param_int_info.emplace("AUTO_RECOGNIZE_BALANCE_INFANTRY_ON", 1);
  this->param_int_info.emplace("INFANTRY3_IS_BALANCE", 0);
  this->param_int_info.emplace("INFANTRY4_IS_BALANCE", 0);
  this->param_int_info.emplace("INFANTRY5_IS_BALANCE", 0);
  // 机械物理量
  this->param_double_info.emplace("BULLET_RADIUS", 8.5e-3);
  this->param_double_info.emplace("BULLET_RESISTANCE_M", 0.0032);
  this->param_double_info.emplace("HERO_BULLET_RESISTANCE_K", 123);
  this->param_double_info.emplace("HERO_BULLET_SIZE", 0);
  this->param_double_info.emplace("BULLET_RESISTANCE_K", 0.022928514188);
  this->param_double_info.emplace("CAMERA_TO_BARREL_Y", -0.055);
  // stm32 接收量控制
  this->param_int_info.emplace("STM32_USING_ANGLE_COMPENSATE", 1);
  // 控制量控制
  this->param_int_info.emplace("CMD_YAW_V_ON", 0);
  this->param_int_info.emplace("CMD_YAW_V_IS_RADIAN", 1);
  // 解算量修正
  this->param_int_info.emplace("PREDICTION_ITERATIONS_NUM", 2);
  this->param_double_info.emplace("SEND_TO_CONTROL_LATENCY", -0.018);
  this->param_double_info.emplace("ADDITIONAL_PREDICTION_TIME", 0.060);
  this->param_double_info.emplace("DEFAULT_CONTROL_TO_FIRE_LATENCY", 0.060);
  this->param_double_info.emplace("AIM_YAW_OFFSET", 0.);
  this->param_double_info.emplace("AIM_PITCH_OFFSET", 0.);
  // 视觉信息源修正
  this->param_double_info.emplace("PNP_DISTANCE_FIXER_A2", 0.);
  this->param_double_info.emplace("NEW_ARMOR_MAX_DIS", 10.);
  // 状态运算参数
  this->param_double_info.emplace("BALANCE_JUDGER_TO_DOUBLE_TIME", 0.2);
  this->param_double_info.emplace("BALANCE_JUDGER_TO_SINGLE_TIME", 15.);
  // 运动运算控制
  this->param_int_info.emplace("TARGET_ROI_ON", 1);
  this->param_double_info.emplace("TARGET_ROI_RECT_Y", 0.5);
  this->param_int_info.emplace("UPDATING_ENEMY_MAX_NUM", 1);
  // 装甲板模型
  this->param_double_info.emplace("MOTION_EKF_Q_X", 0.01);
  this->param_double_info.emplace("MOTION_EKF_Q_V", 100.);
  this->param_double_info.emplace("MOTION_EKF_R_YAW", 1.);
  this->param_double_info.emplace("MOTION_EKF_R_PITCH", 1.5);
  this->param_double_info.emplace("MOTION_EKF_R_DISTANCE_AT_1M", 300.);
  this->param_double_info.emplace("MOTION_AIM_AREA_RATIO", 0.6);
  this->param_double_info.emplace("MOTION_PASSIVE_MODE_AIM_MAX_ERROR", 0.5);
  this->param_double_info.emplace("MOTION_PASSIVE_MODE_AIM_TRACKING_RANGE",
                                  1.0);
  // 全车模型
  this->param_double_info.emplace("TOP_ACTIVE_W", 90.);
  this->param_double_info.emplace("TOP_INACTIVE_W", 70.);
  this->param_double_info.emplace("TOP_SINGLE_ANGLE_R", 150.);
  this->param_double_info.emplace("TOP_DOUBLE_ANGLE_R", 60.);
  this->param_double_info.emplace("TOP_AB_LENGTH_R_AT_1M", 1500.);
  this->param_double_info.emplace("TOP_AB_Z_R_AT_1M", 400.);
  this->param_double_info.emplace("TOP_CENTER_R", 70.);
  this->param_double_info.emplace("TOP_RADIUS_SAMPLING_AREA_RATIO", 0.71);
  this->param_double_info.emplace("TOP_AIM_MAX_ORIENTATION_ANGLE", 0.);
  this->param_double_info.emplace("TOP_AIM_MAX_SWING_ERROR", 3.0);
  this->param_double_info.emplace("TOP_AIM_MAX_OUT_ERROR", 1.0);
  // - 4 板全车模型
  this->param_int_info.emplace("TOP4_ACTIVE_ROTATE", 2);
  // 步兵统一
  this->param_double_info.emplace("INFANTRY_AIM_MAX_ERROR", 1.8);
  // 哨兵统一
  this->param_int_info.emplace("SENTRY_PASSIVE_MODE_ON", 0);
  this->param_double_info.emplace("SENTRY_AIM_MAX_ERROR", 2.5);
  // 前哨站统一
  this->param_int_info.emplace("OUTPOST_TOP_ORIENTATION_SIGN_FIXER_ON", 0);
  // 瞄准校正器
  this->param_int_info.emplace("AIM_CORRECTOR_SAMPLE_ON", 0);
  this->param_int_info.emplace("AIM_CORRECTOR_CORRECTION_ON", 0);
  this->param_double_info.emplace("AIM_CORRECTOR_ERROR_R", 1.);
  // debug 参数 - periodic
  this->param_double_info.emplace("PERIODIC_MONITOR_PERIOD", 1.);
  // - flask
  this->param_double_info.emplace("FLASK_MAP_PIXEL_PER_METER", 100.);
  this->param_int_info.emplace("FLASK_SHOW_PLANE", 0);
  this->param_double_info.emplace("FLASK_PLANE_Z", 0.);
  this->param_int_info.emplace("FLASK_SHOW_CUBE", 0);
  this->param_double_info.emplace("FLASK_CUBE_Z", 0.);
  this->param_double_info.emplace("FLASK_CUBE_DISTANCE", 3.);
  // - RobotCmd
  this->param_int_info.emplace("CMD_DEBUGGER_ON", 0);
  this->param_int_info.emplace("CMD_DEBUGGER_CAR_ID_ON", 0);
  this->param_int_info.emplace("CMD_DEBUGGER_CAR_ID", 0);
  this->param_int_info.emplace("CMD_DEBUGGER_DETECTION_INFO_ON", 0);
  this->param_int_info.emplace("CMD_DEBUGGER_DETECTION_INFO", 0);
  this->param_int_info.emplace("CMD_DEBUGGER_YAW_ON", 0);
  this->param_double_info.emplace("CMD_DEBUGGER_YAW", 0.);
  this->param_int_info.emplace("CMD_DEBUGGER_PITCH_ON", 0);
  this->param_double_info.emplace("CMD_DEBUGGER_PITCH", 0.);
  this->param_int_info.emplace("CMD_DEBUGGER_YAW_V_ON", 0);
  this->param_double_info.emplace("CMD_DEBUGGER_YAW_V", 0.);
  this->param_int_info.emplace("CMD_DEBUGGER_PITCH_V_ON", 0);
  this->param_double_info.emplace("CMD_DEBUGGER_PITCH_V", 0.);
  this->param_int_info.emplace("CMD_DEBUGGER_DIST_ON", 0);
  this->param_double_info.emplace("CMD_DEBUGGER_DIST", 0.);
  this->param_int_info.emplace("CMD_DEBUGGER_SHOOT_ON", 0);
  this->param_int_info.emplace("CMD_DEBUGGER_SHOOT", 0);
  // - RobotStatus
  this->param_int_info.emplace("STATUS_DEBUGGER_ON", 0);
  this->param_int_info.emplace("STATUS_DEBUGGER_BULLET_SPEED_ON", 0);
  this->param_double_info.emplace("STATUS_DEBUGGER_BULLET_SPEED", 28.5);
  this->param_int_info.emplace("STATUS_DEBUGGER_ENEMY_COLOR_ON", 0);
  this->param_int_info.emplace("STATUS_DEBUGGER_ENEMY_COLOR", 0);
  this->param_int_info.emplace("STATUS_DEBUGGER_LAST_SHOOT_AIM_ID_ON", 0);
  this->param_int_info.emplace("STATUS_DEBUGGER_LAST_SHOOT_AIM_ID", 0);
  // - Simulators
  this->param_int_info.emplace("STM32_SIMULATOR_ON", 0);
  this->param_double_info.emplace("STM32_SIMULATOR_SHOOT_INTERVAL", 100e-3);
  // Hero_lob debug
  this->param_int_info.emplace("HERO_LOB_RESTART", 0);
  this->param_int_info.emplace("HERO_LOB_START_STATUS", 0);
  this->param_double_info.emplace("HERO_YAW_OFFSET", 0);
  this->param_double_info.emplace("HERO_AVERAGE_BULLET_SPEED", 0);
  this->param_double_info.emplace("HERO_BULLET_ALPHA", 0);

  for (auto& d : this->param_int_info) {
    this->param_int.emplace(param::create_int_obj(d.first));
    param::find_int_obj(d.first)->value = d.second;
  }
  for (auto& d : this->param_double_info) {
    this->param_double.emplace(param::create_double_obj(d.first));
    param::find_double_obj(d.first)->value = d.second;
  }
  cv::FileStorage reader(CMAKE_DEFINE_PROJECT_DIR "/assets/aimer_param.yml",
                         cv::FileStorage::READ);
  if (reader.isOpened()) {
    try {
      for (auto& d : this->param_int_info) {
        reader[d.first] >> param::find_int_obj(d.first)->value;
      }
      for (auto& d : this->param_double_info) {
        reader[d.first] >> param::find_double_obj(d.first)->value;
      }
    } catch (const std::exception& e) {
      base::print(base::PrintMode::Error, "param",
                  "无法载入 aimer_param.yml, 请检查 yml 文件是否存在.\n");
      std::this_thread::sleep_for(2s);
    }
    base::print(base::PrintMode::Info, "param", "已载入 aimer_param.yml.\n");
  } else {
    std::cerr << "[PARAM] Loading when aimer_param.yml not found, load "
                 "default values.\n";
  }

  this->param_int.emplace(param::create_int_obj("PARAM_LOADED"));
}

void PredictorParameter::update() {
  using namespace std::chrono_literals;

  while (true) {
    bool obj_uncreated = false;
    for (auto& d : this->param_int_info) {
      if (param::find_int_obj(d.first) == nullptr) {
        obj_uncreated = true;
      }
    }
    for (auto& d : this->param_double_info) {
      if (param::find_double_obj(d.first) == nullptr) {
        obj_uncreated = true;
      }
    }
    if (obj_uncreated) {
      std::cerr << "[PARAM] Predictor parameter tried scanning yml, but "
                   "parameter pointers are not created yet. \n";
      std::this_thread::sleep_for(1s);
      continue;
    }
    cv::FileStorage reader(CMAKE_DEFINE_PROJECT_DIR "/assets/aimer_param.yml",
                           cv::FileStorage::READ);
    if (!reader.isOpened()) {
      base::print(
          base::PrintMode::Error, "param",
          "更新时寻找 aimer_param.yml 失败了, 请检查 yml 文件是否存在.\n");
      std::this_thread::sleep_for(1s);
      continue;
    }

    for (auto& d : this->param_int_info) {
      int cache = 0;
      reader[d.first] >> cache;
      if (param::find_int_obj(d.first)->value != cache) {
        int previous = param::find_int_obj(d.first)->value;
        param::find_int_obj(d.first)->value = cache;
        base::print(base::PrintMode::Info, "param",
                    "int 类型参数 \"{}\" 修改: {} -> {}\n", d.first, previous,
                    cache);
      }
    }
    for (auto& d : this->param_double_info) {
      double cache = 0.;
      reader[d.first] >> cache;
      if (param::find_double_obj(d.first)->value != cache) {
        double previous = param::find_double_obj(d.first)->value;
        param::find_double_obj(d.first)->value = cache;
        base::print(base::PrintMode::Info, "param",
                    "double 类型参数 \"{}\" 修改: {} -> {}\n", d.first,
                    previous, cache);
      }
    }

    std::this_thread::sleep_for(1s);
  }
}

void parameter_run() {
  using namespace std::chrono_literals;
  // 创建提供一个全局容器
  std::unique_ptr<PredictorParameter> predictor_parameter =
      std::make_unique<PredictorParameter>();
  predictor_parameter->load();
  while (true) {
    predictor_parameter->update();
    std::this_thread::sleep_for(1s);
  }
}

}  // namespace param
}  // namespace aimer
