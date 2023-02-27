#include "aimer/auto_aim/predictor/motion/enemy_model.hpp"

#include <cstddef>
#include <iostream>
#include <memory>
#include <vector>

#include <fmt/color.h>
#include <fmt/format.h>

#include "aimer/auto_aim/predictor/motion/motion_model.hpp"
#include "aimer/auto_aim/predictor/motion/top_model.hpp"
#include "aimer/base/debug/debug.hpp"

namespace aimer {

// 禁止使用 extern 声明跨文件全局变量，否则将导致构造顺序的不确定
const std::unordered_map<aimer::EnemyType, aimer::ModelType> ENEMY_TO_MODEL = {
    {aimer::EnemyType::SENTRY, aimer::ModelType::SENTRY},
    {aimer::EnemyType::HERO, aimer::ModelType::INFANTRY},
    {aimer::EnemyType::ENGINEER, aimer::ModelType::INFANTRY},
    {aimer::EnemyType::INFANTRY, aimer::ModelType::INFANTRY},
    {aimer::EnemyType::BALANCE_INFANTRY, aimer::ModelType::BALANCE_INFANTRY},
    {aimer::EnemyType::OUTPOST, aimer::ModelType::OUTPOST},
    {aimer::EnemyType::CRYSTAL_BIG, aimer::ModelType::STATUE},
    {aimer::EnemyType::CRYSTAL_SMALL, aimer::ModelType::STATUE}};

const std::unordered_map<aimer::ModelType, double> MODEL_TOP_CREDIT_TIME = {
    {aimer::ModelType::INFANTRY, 0.1},
    {aimer::ModelType::OUTPOST, 1.},
    {aimer::ModelType::BALANCE_INFANTRY, 0.5}};

// 注意 id 线程生命周期是 0.1s，而滤波器线程的存活则完全取决于 credit
const std::unordered_map<aimer::ModelType, double> MODEL_MOTION_CREDIT_TIME = {
    {aimer::ModelType::SENTRY, 0.08},
    {aimer::ModelType::INFANTRY, 0.08},
    {aimer::ModelType::OUTPOST, 0.08},
    {aimer::ModelType::BALANCE_INFANTRY, 0.08},
    {aimer::ModelType::STATUE, 0.08}};

/** @class Sentry */

Sentry::Sentry(aimer::CoordConverter* const converter,
               aimer::EnemyState* const state)
    : converter(converter),
      state(state),
      motion_model(
          converter,
          state,
          aimer::MODEL_MOTION_CREDIT_TIME.at(aimer::ModelType::SENTRY)) {}

bool Sentry::alive() const {
  return this->get_model_type() ==
         aimer::ENEMY_TO_MODEL.at(this->state->get_enemy_type());
}

void Sentry::update() {
  // 常规运动模块更新
  this->motion_model.update();
}

// send 的 shoot 也需要我处理
aimer::AimInfo Sentry::get_aim() const {
  return this->motion_model.get_aim(aimer::MotionModel::GetAimOption(
      aimer::param::find_int_param("SENTRY_PASSIVE_MODE_ON") == 1));
}

void Sentry::draw_aim(cv::Mat& img) const {
  this->motion_model.draw_aim(img);
}

/** @class Infantry */
// number 是 Enemy 的序号，所以在 Enemy 的外部
// 会陀螺的目标

Infantry::Infantry(aimer::CoordConverter* const converter,
                   aimer::EnemyState* const state)
    : converter(converter),
      state(state),
      motion_model(
          converter,
          state,
          aimer::MODEL_MOTION_CREDIT_TIME.at(aimer::ModelType::INFANTRY)),
      top4_model(converter,
                 state,
                 aimer::MODEL_TOP_CREDIT_TIME.at(aimer::ModelType::INFANTRY)) {}

bool Infantry::alive() const {
  return this->get_model_type() ==
         aimer::ENEMY_TO_MODEL.at(this->state->get_enemy_type());
}

void Infantry::update() {
  this->motion_model.update();
  this->top4_model.update();
}

// send 的 shoot 也需要我处理
aimer::AimInfo Infantry::get_aim() const {
  aimer::AimInfo aim = aimer::AimInfo::idle();
  if (this->top4_model.active()) {
    aim = this->top4_model.get_limit_aim();
    aim.info |= aimer::AimInfo::TOP;  // 反陀螺激活
  } else {
    aim = this->motion_model.get_aim(aimer::MotionModel::GetAimOption(false));
  }
  return aim;
}

void Infantry::draw_aim(cv::Mat& img) const {
  this->motion_model.draw_aim(img);
  this->top4_model.draw_aim(img);
  if (this->top4_model.active()) {
    aimer::debug::flask_aim
        << fmt::format("{}: Top Act", this->state->get_number());
  }
}

/** @class BalanceInfantry */

BalanceInfantry::BalanceInfantry(aimer::CoordConverter* const converter,
                                 aimer::EnemyState* const state)
    : converter(converter),
      state(state),
      // C++ 的规则真是一团糟！
      motion_model{aimer::MotionModel(converter,
                                      state,
                                      aimer::MODEL_MOTION_CREDIT_TIME.at(
                                          aimer::ModelType::BALANCE_INFANTRY))},
      top2_model{aimer::top::SimpleTopModel(
          /*converter=*/converter,
          /*state=*/state,
          /*armor_cnt=*/2,
          /*jump_angle=*/M_PI / 8.,
          /*min_active_rotate*/ 2,
          /*credit_time=*/
          aimer::MODEL_TOP_CREDIT_TIME.at(aimer::ModelType::BALANCE_INFANTRY),
          /*cons=*/aimer::top::OrientationSignFixer::Constructor(0., 0., 0.))} {
}

bool BalanceInfantry::alive() const {
  return this->get_model_type() ==
         aimer::ENEMY_TO_MODEL.at(this->state->get_enemy_type());
}

void BalanceInfantry::update() {
  this->motion_model.update();
  this->top2_model.update(aimer::top::SimpleTopModel::UpdateOption(false));
}

aimer::AimInfo BalanceInfantry::get_aim() const {
  aimer::AimInfo aim = aimer::AimInfo::idle();
  if (this->top2_model.active()) {
    aim = this->top2_model.get_limit_aim();
    aim.info |= aimer::AimInfo::TOP;  // 反陀螺激活信息
  } else {
    aim = this->motion_model.get_aim(aimer::MotionModel::GetAimOption(false));
  }
  return aim;
}

void BalanceInfantry::draw_aim(cv::Mat& img) const {
  this->motion_model.draw_aim(img);
  this->top2_model.draw_aim(img);
  if (this->top2_model.active()) {
    aimer::debug::flask_aim
        << fmt::format("{}: Top Act", this->state->get_number());
  }
}

/** @class Outpost */

Outpost::Outpost(aimer::CoordConverter* const converter,
                 aimer::EnemyState* const state)
    : converter(converter),
      state(state),
      motion_model(
          converter,
          state,
          aimer::MODEL_MOTION_CREDIT_TIME.at(aimer::ModelType::OUTPOST)),
      top3_model(
          converter,
          state,
          /*armor_cnt=*/3,
          /*jump_angle=*/M_PI / 8.,
          /*min_active_rotate=*/1,
          aimer::MODEL_TOP_CREDIT_TIME.at(aimer::ModelType::OUTPOST),
          aimer::top::OrientationSignFixer::Constructor(2., 0.1, M_PI / 8.)) {}

bool Outpost::alive() const {
  return this->get_model_type() ==
         aimer::ENEMY_TO_MODEL.at(this->state->get_enemy_type());
}

void Outpost::update() {
  this->motion_model.update();
  this->top3_model.update(aimer::top::SimpleTopModel::UpdateOption(
      aimer::param::find_int_param("OUTPOST_TOP_ORIENTATION_SIGN_FIXER_ON") ==
      1));
}

// send 的 shoot 也需要我处理
aimer::AimInfo Outpost::get_aim() const {
  aimer::AimInfo aim = aimer::AimInfo::idle();
  if (this->top3_model.active()) {
    aim = this->top3_model.get_limit_aim();
    aim.info |= aimer::AimInfo::TOP;  // 反陀螺激活
  } else {
    aim = this->motion_model.get_aim(aimer::MotionModel::GetAimOption(false));
  }
  return aim;
}

void Outpost::draw_aim(cv::Mat& img) const {
  this->motion_model.draw_aim(img);
  this->top3_model.draw_aim(img);
  if (this->top3_model.active()) {
    aimer::debug::flask_aim
        << fmt::format("{}: Top Act", this->state->get_number());
  }
}

/** @class Statue */

Statue::Statue(aimer::CoordConverter* const converter,
               aimer::EnemyState* const state)
    : converter(converter),
      state(state),
      motion_model(
          converter,
          state,
          aimer::MODEL_MOTION_CREDIT_TIME.at(aimer::ModelType::STATUE)) {}

bool Statue::alive() const {
  return this->get_model_type() ==
         aimer::ENEMY_TO_MODEL.at(this->state->get_enemy_type());
}

void Statue::update() {
  // 常规运动模块更新
  this->motion_model.update();
}

// send 的 shoot 也需要我处理
aimer::AimInfo Statue::get_aim() const {
  aimer::AimInfo aim = this->motion_model.get_aim(
      aimer::MotionModel::GetAimOption(false));  // converter means now
  return aim;
}

void Statue::draw_aim(cv::Mat& img) const {
  this->motion_model.draw_aim(img);
}

const std::unordered_map<
    aimer::ModelType,
    std::function<std::unique_ptr<aimer::EnemyModelInterface>(
        aimer::CoordConverter* const converter,
        aimer::EnemyState* const state)>>
    MODEL_MAP = {
        {aimer::ModelType::SENTRY,
         [](aimer::CoordConverter* const converter,
            aimer::EnemyState* const state) {
           return std::make_unique<Sentry>(converter, state);
         }},
        {aimer::ModelType::INFANTRY,
         [](aimer::CoordConverter* const converter,
            aimer::EnemyState* const state) {
           return std::make_unique<Infantry>(converter, state);
         }},
        {aimer::ModelType::BALANCE_INFANTRY,
         [](aimer::CoordConverter* const converter,
            aimer::EnemyState* const state) {
           return std::make_unique<BalanceInfantry>(converter, state);
         }},
        {aimer::ModelType::OUTPOST,
         [](aimer::CoordConverter* const converter,
            aimer::EnemyState* const state) {
           return std::make_unique<Outpost>(converter, state);
         }},
        {aimer::ModelType::STATUE, [](aimer::CoordConverter* const converter,
                                      aimer::EnemyState* const state) {
           return std::make_unique<Statue>(converter, state);
         }}};

/** @class EnemyModelFactory */

std::unique_ptr<aimer::EnemyModelInterface> EnemyModelFactory::create_model(
    aimer::CoordConverter* const converter,
    aimer::EnemyState* const state) {
  return aimer::MODEL_MAP.at(aimer::ENEMY_TO_MODEL.at(state->get_enemy_type()))(
      converter, state);
}
}  // namespace aimer
