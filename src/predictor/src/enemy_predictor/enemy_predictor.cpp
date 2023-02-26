#include "aimer/auto_aim/predictor/enemy_predictor/enemy_predictor.hpp"

#include <algorithm>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "./common/common.hpp"
#include "aimer/auto_aim/base/defs.hpp"
#include "aimer/auto_aim/predictor/aim/aim_corrector.hpp"
#include "aimer/auto_aim/predictor/enemy/armor_identifier.hpp"
#include "aimer/auto_aim/predictor/pnp/pnp.hpp"
#include "aimer/base/armor_defs.hpp"
#include "aimer/base/debug/debug.hpp"
#include "aimer/base/math/math.hpp"
#include "aimer/base/robot/coord_converter.hpp"
#include "aimer/param/parameter.hpp"
#include "core_io/robot.hpp"

// 不在自瞄也要进行所有 kalman，只是不发送电控指令，且选敌逻辑有区别

// 系统信息，本质不同的坐标变换或者需要系统参数的变换的都需我的实例
// 目前，涉及打击决策或者时间线程管理的类都需要获得它的指针
namespace aimer {

// 决策参数
const std::unordered_map<aimer::EnemyType, double> ENEMY_KEEP_AS_TARGET_TIME{
    {aimer::EnemyType::SENTRY, 0.1},
    {aimer::EnemyType::HERO, 0.1},
    {aimer::EnemyType::ENGINEER, 0.1},
    {aimer::EnemyType::INFANTRY, 0.1},
    // 平衡步兵装甲板往往会从视野里消失
    {aimer::EnemyType::BALANCE_INFANTRY, 0.5},
    {aimer::EnemyType::OUTPOST, 0.5},
    {aimer::EnemyType::CRYSTAL_BIG, 0.1},
    {aimer::EnemyType::CRYSTAL_SMALL, 0.1}};

const std::unordered_map<aimer::EnemyType, bool> ENEMY_TARGET_ROI_OPTION{
    {aimer::EnemyType::SENTRY, true},
    {aimer::EnemyType::HERO, false},
    {aimer::EnemyType::ENGINEER, false},
    {aimer::EnemyType::INFANTRY, false},
    {aimer::EnemyType::BALANCE_INFANTRY, false},
    {aimer::EnemyType::OUTPOST, true},
    {aimer::EnemyType::CRYSTAL_BIG, false},
    {aimer::EnemyType::CRYSTAL_SMALL, false}};

// 没什么用，当未捕获任何目标时，旧目标的保持时间（会打必须要那个模型被信任）
const double TARGET_MEMORIZING_TIME = 5.;

namespace debug {
auto catch_duration(const std::string& name) {
  auto fmt_pair = aimer::debug::start_end_time_to_fmt_pair(
      aimer::debug::register_timer.get_and_register(name));
  aimer::debug::auto_aim_page()->sub("预测器效率").sub(fmt_pair.first).get() =
      fmt_pair.second;
}
}  // namespace debug

/**
 * @brief Construct a new Target Catcher:: Target Catcher object
 *
 * @param converter
 * @param states
 */
TargetCatcher::TargetCatcher(aimer::CoordConverter* const converter,
                             aimer::EnemyState* const states)
    : converter(converter), states{states} {}

/**
 * @brief
 *
 * @param target_number
 */
void TargetCatcher::catch_target(const int& target_number) {
  if (this->target_number == -1) {
    this->target_number = target_number;
  } else if (this->target_number != target_number) {
    // target_number != -1
    // 可根据不同车的类型改变 Keep 时间
    if (this->converter->get_img_t() - this->target_caught_t >
        aimer::ENEMY_KEEP_AS_TARGET_TIME.at(
            this->states[this->target_number].get_enemy_type())) {
      this->target_number = target_number;
    }
  }
  if (this->target_number == target_number) {
    this->target_caught_t = this->converter->get_img_t();
  }
}

int TargetCatcher::get_target() const {
  if (this->converter->get_img_t() - this->target_caught_t >
      aimer::TARGET_MEMORIZING_TIME) {
    return -1;
  }
  return this->target_number;
}

/**
 * @class EnemyPredictor
 *
 */
EnemyPredictor::EnemyPredictor()
    : enemy_states{
          aimer::EnemyState(0, aimer::EnemyType::SENTRY),
          aimer::EnemyState(1, aimer::EnemyType::HERO),
          aimer::EnemyState(2, aimer::EnemyType::ENGINEER),
          aimer::EnemyState(3, aimer::EnemyType::INFANTRY),
          // 运行时可能会更新 enemy_type
          aimer::EnemyState(4, aimer::EnemyType::INFANTRY),
          aimer::EnemyState(5, aimer::EnemyType::INFANTRY),
          aimer::EnemyState(6, aimer::EnemyType::OUTPOST),
          aimer::EnemyState(7, aimer::EnemyType::CRYSTAL_BIG),
          aimer::EnemyState(8, aimer::EnemyType::CRYSTAL_SMALL),
      },
      light{aimer::LightManager(&this->converter)},
      target_catcher{aimer::TargetCatcher(&this->converter, this->enemy_states)},
      aim_corrector{aimer::aim::AimCorrector(&this->converter)} {
  for (int i = 0; i <= aimer::MAX_ENEMY_NUMBER; ++i) {
    this->enemy_models[i] = this->model_factory.create_model(
        &this->converter, &this->enemy_states[i]);
  }
}

void EnemyPredictor::update_database(const aimer::DetectionResult& data) {
  this->converter.update(data.img, data.q, data.timestamp);

  aimer::debug::auto_aim_page()->sub("机器人状态").sub("图像帧").get() =
      fmt::format("{}", this->converter.get_frame());
  aimer::debug::auto_aim_page()->sub("机器人状态").sub("图像时间").get() =
      fmt::format("{:.3f}", this->converter.get_img_t());

  aimer::debug::auto_aim_page()
      ->sub("识别和分类")
      .sub("网络发布装甲板数量")
      .get() = fmt::format("{}", data.armors.size());

  aimer::debug::auto_aim_page()->sub("机器人状态").sub("电控_运行模式").get() =
      std::to_string(static_cast<int>(
          this->converter.get_robot_status_ref().program_mode)) +
      " " + ([&]() -> std::string {
        switch (this->converter.get_robot_status_ref().program_mode) {
          case ::ProgramMode::AUTOAIM:
            return "AUTOAIM 自瞄";
          case ::ProgramMode::MANUAL:
            return "MANUAL 手动";
          case ::ProgramMode::ENERGY_HIT:
            return "ENERGY_HIT 能量机关";
          case ::ProgramMode::NOT_RECEIVED:
            return "NOT_RECEIVED 未曾收到";
          default:
            return "OTHER";
        }
      }());

  aimer::debug::auto_aim_page()
      ->sub("机器人状态")
      .sub("相机_z_轴_yaw_imu_系")
      .get() = fmt::format("{:.2f} /*向左为正*/",
                           this->converter.get_camera_z_i_yaw() / M_PI * 180.);

  aimer::debug::auto_aim_page()
      ->sub("机器人状态")
      .sub("相机_z_轴_pitch_imu_系")
      .get() =
      fmt::format("{:.2f} /*向上为正*/",
                  this->converter.get_camera_z_i_pitch() / M_PI * 180.);

  aimer::debug::auto_aim_page()->sub("机器人状态").sub("电控_敌人颜色").get() =
      this->converter.get_robot_status_ref().enemy_color == ::EnemyColor::BLUE
          ? "BLUE0"
          : "RED1";

  aimer::debug::auto_aim_page()->sub("机器人状态").sub("电控_子弹速度").get() =
      fmt::format("{:.5f}",
                  this->converter.get_robot_status_ref().bullet_speed);

  aimer::debug::auto_aim_page()
      ->sub("机器人状态")
      .sub("电控_yaw_轴补偿")
      .get() =
      fmt::format("原始: {:.2f} 视觉认为他使用: {:.2f}",
                  this->converter.get_robot_status_ref().yaw_compensate,
                  this->converter.get_yaw_compensate() / M_PI * 180.);

  aimer::debug::auto_aim_page()
      ->sub("机器人状态")
      .sub("电控_pitch_轴补偿")
      .get() =
      fmt::format("原始: {:.2f} 视觉认为他使用: {:.2f}",
                  this->converter.get_robot_status_ref().pitch_compensate,
                  this->converter.get_pitch_compensate() / M_PI * 180.);

  aimer::debug::auto_aim_page()
      ->sub("机器人状态")
      .sub("电控_上一射出子弹的瞄准编号")
      .get() = fmt::format(
      "{}", this->converter.get_robot_status_ref().last_shoot_aim_id);

  std::vector<aimer::DetectedArmor> fixed_vec;
  for (auto& d : data.armors) {
    aimer::DetectedArmor fixed = this->detected_fixer.fixed(d);
    if (fixed.number >= 0 && fixed.number <= aimer::MAX_ENEMY_NUMBER) {
      fixed_vec.push_back(fixed);
    }
  }
  std::vector<aimer::ArmorInfo> armor_info_vec;
  for (auto& d : fixed_vec) {
    aimer::ArmorInfo armor_info = aimer::detected_to_info(
        d, this->enemy_states[d.number].get_sample_armor_ref(),
        this->enemy_states[d.number].get_armor_pitch(), &this->converter);
    // 根据一些外部特征修复装甲板数据，把修复结果塞入数组
    if (armor_info.valid()) {
      armor_info_vec.push_back(armor_info);
    }
  }

  // 把装甲板粘到现有的线程上。修复闪烁颜色，获取出现的 id
  // 此处如果有线程删不干净十分危险
  this->light.update(armor_info_vec);
  // 把 light 生产的带 id
  // 的装甲板分在各个车的容器里，更新每个车的状态(enemy_state) 数据
  std::vector<aimer::ArmorData> all_armors = this->light.get_all_armors();
  std::vector<aimer::ArmorData> sub_armors[aimer::MAX_ENEMY_NUMBER + 1];
  for (auto& d : all_armors) {
    sub_armors[d.info.detected.number].push_back(d);
  }
  // 下面所有车应强制更新，否则会导致上一帧数据残留
  for (int i = 0; i <= aimer::MAX_ENEMY_NUMBER; ++i) {
    this->enemy_states[i].update(sub_armors[i], this->converter.get_img_t(),
                                 this->converter.get_frame());
  }
}

void EnemyPredictor::check_models() {
  for (int i = 3; i <= 5; ++i) {
    if (!this->enemy_models[i]->alive()) {
      // 智能指针指向 nullptr 可以 销毁运动模型
      this->enemy_models[i] = nullptr;
      this->enemy_models[i] = this->model_factory.create_model(
          &this->converter, &this->enemy_states[i]);
    }
  }
}

void EnemyPredictor::get_all_armors() {
  std::vector<aimer::ArmorData> all_armors;
  for (int i = 0; i <= aimer::MAX_ENEMY_NUMBER; ++i) {
    const std::vector<aimer::ArmorData>& armors =
        this->enemy_states[i].get_armor_data_ref();
    for (auto& d : armors)
      all_armors.push_back(d);
  }
  // 根据它们到准心的距离排序
  std::sort(all_armors.begin(), all_armors.end(),
            [this](const aimer::ArmorData& d1, const aimer::ArmorData& d2) {
              cv::Point2f img_center =
                  cv::Point2f(this->converter.get_img_ref().cols / 2.f,
                              this->converter.get_img_ref().rows / 2.f);
              return aimer::math::get_dis(d1.info.center(), img_center) <
                     aimer::math::get_dis(d1.info.center(), img_center);
            });
  this->all_armors = all_armors;
}

void EnemyPredictor::get_target() {
  int last_tracking_target =
      this->converter.get_robot_status_ref().program_mode ==
              ::ProgramMode::AUTOAIM
          ? this->target_catcher.get_target()
          : -1;
  // 可能成为目标的敌人序号
  std::vector<aimer::ArmorData> candidate_targets;
  for (auto& armor : this->all_armors) {
    // 若序号是上一帧被电机追踪的序号，则加入 candidate
    if (armor.info.detected.number == last_tracking_target) {
      candidate_targets.push_back(armor);
      continue;
    }
    // 若上一帧未追踪，则根据它是否在该兵种的感兴趣区域加入 candidate
    cv::Rect2f roi_rate =
        aimer::param::find_int_param("TARGET_ROI_ON") == 1 &&
                aimer::ENEMY_TARGET_ROI_OPTION.at(
                    this->enemy_states[armor.info.detected.number]
                        .get_enemy_type())
            ? ([]() {
                float y =
                    float(aimer::param::find_double_param("TARGET_ROI_RECT_Y"));
                return cv::Rect2f(/*x=*/0.f, /*y=*/y, /*width=*/1.f,
                                  /*height=*/1.f - y);
              }())
            : cv::Rect2f(0.f, 0.f, 1.f, 1.f);
    cv::Rect2f roi =
        cv::Rect2f(this->converter.get_img_ref().cols * roi_rate.x,
                   this->converter.get_img_ref().rows * roi_rate.y,
                   this->converter.get_img_ref().cols * roi_rate.width,
                   this->converter.get_img_ref().rows * roi_rate.height);
    if (!(roi & aimer::math::get_box(armor.info.detected.pts)).empty()) {
      candidate_targets.push_back(armor);
    }
  }
  // 根据 candidate_targets 选择更新
  if (this->converter.get_robot_status_ref().program_mode !=
      ::ProgramMode::AUTOAIM) {
    // 非自瞄模式下，如果有数据，捕获视野中心最近的（简称当前帧最优）
    // 捕获并不一定立即成为目标，为了打击平衡、前哨站，
    // 我们会等待旧目标一个时间后再放弃之
    if (!candidate_targets.empty()) {
      this->target_catcher.catch_target(
          candidate_targets.front().info.detected.number);
    }
  } else {
    // 自瞄模式下
    // 如果没有目标，尝试选择最优的；
    if (last_tracking_target == -1) {
      if (!candidate_targets.empty()) {
        this->target_catcher.catch_target(
            candidate_targets.front().info.detected.number);
      }
    } else {  // 如果有目标: - 若目标无数据，就尝试抓取当前帧最优数据
      if (this->enemy_states[last_tracking_target]
              .get_armor_data_ref()
              .empty()) {
        if (!candidate_targets.empty()) {
          this->target_catcher.catch_target(
              candidate_targets.front().info.detected.number);
        }
      } else {  // - 若目标有数据，则抓取该目标
        this->target_catcher.catch_target(last_tracking_target);
      }
    }
  }
}

void EnemyPredictor::update_models() {
  std::set<int> update_list;
  // 更新目标运动模型（节约时间，不是更新所有模型）
  // 优先为 target 更新。更新条件：target 存在数据或在持续追踪名单中
  // 这个条件是一个重要条件
  // 该条件允许加入 tracking_list，且允许打击
  // 虽然持续追踪时且无数据时，它不会更新，但是要画（本质上是加入可能追踪列表）
  // 本质上，所有目标均可在 lost 后保留 2
  // 秒，以便在多个敌人同时出现（如相机遮挡）时打击保留的目标
  // 而是云台否要追踪目标，则取决于目标是否有数据（或是否是无偿追踪目标）
  if (int target = this->target_catcher.get_target();
      target != -1 &&
      (!this->enemy_states[target].get_armor_data_ref().empty()) &&
      update_list.size() <
          static_cast<std::size_t>(
              aimer::param::find_int_param("UPDATING_ENEMY_MAX_NUM"))) {
    update_list.insert(target);
  }

  // 在剩下的模型中，优先选择最近的更新
  for (auto& d : this->all_armors) {
    if (update_list.find(d.info.detected.number) == update_list.end() &&
        update_list.size() <
            static_cast<std::size_t>(
                aimer::param::find_int_param("UPDATING_ENEMY_MAX_NUM"))) {
      update_list.insert(d.info.detected.number);
    }
  }
  for (auto& number : update_list) {
    this->enemy_models[number]->update();
  }
  this->debug_update_list = update_list;
}

void EnemyPredictor::update_aim_error() {
  if (aimer::param::find_int_param("STM32_SIMULATOR_ON") == 1) {
    this->aim_corrector.update_bullet_id(
        this->stm32_shoot.get_last_shoot_id(this->converter.get_img_t()));
  } else {
    // 更新子弹预估器的状态
    this->aim_corrector.update_bullet_id(
        this->converter.get_robot_status_ref().last_shoot_aim_id);
  }
  if (aimer::param::find_int_param("AIM_CORRECTOR_SAMPLE_ON") == 1) {
    this->aim_corrector.sample_aim_errors();
  }

  {
    Eigen::Vector2d aim_correction = this->aim_corrector.get_aim_error();

    aimer::debug::auto_aim_page()->sub("预测器信息").sub("瞄准校正数据").get() =
        fmt::format(
            "估计偏差: {:.2f} | {:.2f} 是否使用: {}",
            aimer::math::rad_to_deg(aim_correction(0, 0)),
            aimer::math::rad_to_deg(aim_correction(1, 0)),
            aimer::param::find_int_param("AIM_CORRECTOR_CORRECTION_ON") == 1);
  }
}

aimer::AimInfo EnemyPredictor::get_aim() {
  const int target = this->target_catcher.get_target();
  aimer::debug::auto_aim_page()->sub("预测器信息").sub("瞄准敌人编号").get() =
      fmt::format("{}", target);
  if (target == -1)
    return aimer::AimInfo::idle();

  const Eigen::Vector2d aim_correction =
      aimer::param::find_int_param("AIM_CORRECTOR_CORRECTION_ON") == 1
          ? this->aim_corrector.get_aim_error()
          : Eigen::Vector2d(0., 0.);  // 实际 - 理想
  const aimer::AimInfo aim = [&]() {
    aimer::AimInfo aim = this->enemy_models[target]->get_aim();
    if (aim.shoot == ::ShootMode::IDLE)
      return aim;
    aim.ypd.yaw += -aim_correction(0, 0);
    aim.ypd.pitch += -aim_correction(1, 0);
    return aim;
  }();
  if (aim.shoot == ::ShootMode::IDLE)
    return aim;

  this->aim_id_cnt += 1;
  this->stm32_shoot.add(this->aim_id_cnt, this->converter.get_img_t());
  this->aim_corrector.add_aim(aimer::aim::IdTLatencyAimCorrection{
      this->aim_id_cnt, this->converter.get_img_t(),
      this->converter.get_img_to_predict_latency(), aim, aim_correction});

  this->desired_yaw_average.update(
      aim.ypd.yaw, this->converter.get_img_t(),
      aimer::param::find_double_param("PERIODIC_MONITOR_PERIOD"));
  aimer::debug::auto_aim_page()
      ->sub("预测器信息")
      .sub("我希望还需转动_yaw")
      .get() = fmt::format(
      "{:.2f} | {:.2f} 秒内均值: {:.2f}", -aim.ypd.yaw / M_PI * 180.,
      aimer::param::find_double_param("PERIODIC_MONITOR_PERIOD"),
      -aimer::math::rad_to_deg(this->desired_yaw_average.get()));
  aimer::debug::auto_aim_page()
      ->sub("预测器信息")
      .sub("我希望还需转动_pitch")
      .get() = fmt::format("{:.2f}", -aim.ypd.pitch / M_PI * 180.);
  return aim;
}

::RobotCmd EnemyPredictor::get_cmd_by_aim(const aimer::AimInfo& aim) {
  ::RobotCmd cmd = {};

  cmd.shoot = ::ShootMode::IDLE;  // target not found
  cmd.detection_info &= 0;        // 清零
  for (int i = 0; i <= aimer::MAX_ENEMY_NUMBER; ++i) {
    if (!this->enemy_states[i].get_armor_data_ref().empty()) {
      cmd.detection_info |= (1 << i);
    }
  }
  cmd.car_id = 15;  // convention: 15 means no target number

  if (aim.shoot == ::ShootMode::IDLE)
    return cmd;

  // 射击模式 跟随打击: SHOOT_NOW 只跟随: TRACKING
  // 只发包: IDLE（意味着其他信息都不会被使用）
  cmd.aim_id = this->aim_id_cnt;
  cmd.shoot = aim.shoot;
  // 打击目标的编号（装甲板上数字）
  cmd.car_id = this->target_catcher.get_target();
  // aim 的 ypd 是相机右手系 ypd，yaw 向右为正，pitch 向下为正。
  // 电控用的坐标系则相反，
  // send.yaw 表示电控应该控制云台向左（俯视逆时针）转 send.yaw
  // 度。所有车电控均接收角度制
  cmd.yaw = -float((aim.ypd.yaw + aimer::param::find_double_param(
                                      "ADDITIONAL_PREDICTION_TIME") *
                                      aim.ypd_v.yaw) /
                   M_PI * 180.);
  // 表示电控应该控制云台向上转 send.pitch degree
  cmd.pitch = -float((aim.ypd.pitch + aimer::param::find_double_param(
                                          "ADDITIONAL_PREDICTION_TIME") *
                                          aim.ypd_v.pitch) /
                     M_PI * 180.);
  // 目标距离
  cmd.dist = float(aim.ypd.dis);  // of no use
  // 额外表示锁定了的目是否反陀螺
  if (aim.info & (aimer::AimInfo::TOP)) {
    cmd.detection_info |= (1 << (aimer::MAX_ENEMY_NUMBER + 1));
  }
  // 发送目标当前的相机坐标系 yaw 的速度，不同车使用的角度单位不同
  if (aimer::param::find_int_param("CMD_YAW_V_ON") == 1) {
    if (aimer::param::find_int_param("CMD_YAW_V_IS_RADIAN") == 1) {  // 弧度制
      cmd.yaw_v = -float(aim.ypd_v.yaw);
      cmd.pitch_v = -float(aim.ypd_v.pitch);
    } else {
      cmd.yaw_v = -float(aim.ypd_v.yaw / M_PI * 180.);  // 角度制
      cmd.pitch_v = -float(aim.ypd_v.pitch / M_PI * 180.);
    }
  } else {
    cmd.yaw_v = 0.;
    cmd.pitch_v = 0.;
  }
  // 参数表可强制修改发送给电控的命令
  if (aimer::param::find_int_param("CMD_DEBUGGER_ON") == 1) {
    this->cmd_debugger.fix(cmd);
  }

  {
    aimer::debug::auto_aim_page()
        ->sub("预测器信息")
        .sub("发送给电控_yaw")
        .get() = fmt::format("{:.2f}", cmd.yaw);
    aimer::debug::auto_aim_page()
        ->sub("预测器信息")
        .sub("发送给电控_pitch")
        .get() = fmt::format("{:.2f}", cmd.pitch);
    aimer::debug::auto_aim_page()
        ->sub("预测器信息")
        .sub("发送给电控_shoot")
        .get() =
        fmt::format("{}", int(cmd.shoot)) + " " + ([&]() -> std::string {
          switch (cmd.shoot) {
            case ::ShootMode::TRACKING:
              return "跟随且不发弹";
            case ::ShootMode::SHOOT_NOW:
              return "跟随并要求发弹";
            case ::ShootMode::IDLE:
              return "空闲状态，自由活动枪口";
          }
        }());
    if (cmd.shoot == ::ShootMode::SHOOT_NOW) {
      this->cmd_last_shoot_t = this->converter.get_img_t();
    }
    aimer::debug::auto_aim_page()
        ->sub("预测器信息")
        .sub("最近发弹指令时间")
        .get() = fmt::format("{:.2f}", this->cmd_last_shoot_t);
  }
  return cmd;
}

::RobotCmd EnemyPredictor::predict(const aimer::DetectionResult& data) {
  aimer::debug::process_timer.process_begin();
  aimer::debug::register_timer.get_and_register("开始预测");

  aimer::debug::flask_aim.clear();
  aimer::debug::flask_map.clear();
  // 数据库更新
  this->update_database(data);

  this->check_models();

  this->get_all_armors();
  this->get_target();

  this->update_models();

  // 捕获本地时间以计算延迟
  // 在此之前，禁用任何 predict_to_aim!
  this->converter.catch_predict_timestamp();

  aimer::debug::catch_duration("更新模型后");

  this->update_aim_error();

  aimer::debug::catch_duration("瞄准采样后");

  // 获取发送给电控的信息
  const AimInfo aim = this->get_aim();
  this->debug_aim = aim;
  const ::RobotCmd cmd = this->get_cmd_by_aim(aim);
  this->debug_cmd = cmd;

  aimer::debug::catch_duration("计算瞄准点和发送量后");

  this->processing_time_recorder.update(
      aimer::debug::process_timer.get_process_time(),
      this->converter.get_img_t(),
      aimer::param::find_double_param("PERIODIC_MONITOR_PERIOD"));

  aimer::debug::auto_aim_page()
      ->sub("预测器效率")
      .sub("近期最大运行时间")
      .get() =
      fmt::format("于 {:.2f} 秒内: {:.3f} 毫秒",
                  aimer::param::find_double_param("PERIODIC_MONITOR_PERIOD"),
                  this->processing_time_recorder.get() * 1e3);

  // 计算末端延迟，用于未来估计
  this->converter.catch_send_timestamp();
  return cmd;  // 不影响信息发送
}

cv::Mat EnemyPredictor::draw_aim(const cv::Mat& img) {
  cv::Mat aim_mat = img.clone();
  // 时间、帧数、发送给电控的 cmd
  {
    // 图像中心
    cv::circle(aim_mat, {640, 384}, 6, cv::Scalar{220, 220, 220}, 3);

    // 相机 z 轴在图像中的位置并非图像中央，绿色
    cv::circle(aim_mat, this->converter.pc_to_pu({0., 0., 1.}), 6,
               cv::Scalar{0, 180, 0}, 3);
  }

  // 输出平衡机器人列表
  {
    std::string balance_str = "";
    for (int i = 3; i <= 5; ++i) {
      if (this->enemy_states[i].get_enemy_type() ==
          aimer::EnemyType::BALANCE_INFANTRY) {
        balance_str += fmt::format("{} ", i);
      }
    }
    aimer::debug::auto_aim_page()
        ->sub("机器人状态")
        .sub("哪些敌人被认为平衡")
        .get() = balance_str;
  }

  // 绘制模拟发射的子弹
  {
    std::vector<aimer::aim::IdCircle> bullets =
        this->aim_corrector.get_circles();
    for (auto& bullet : bullets) {
      aimer::debug::flask_aim << aimer::debug::FlaskPoint(
          bullet.circle.center, {0, 0, 255}, bullet.circle.r, 2);
      aimer::debug::flask_aim << aimer::debug::FlaskText(
          std::to_string(bullet.id),
          {bullet.circle.center.x + 20.f, bullet.circle.center.y}, {0, 0, 255},
          0.8);
      // aimer::debug::flask_map << aimer::debug::pos_to_map_point(bullet.pos,
      //                                                     {0, 0, 255}, 4,
      //                                                     -1);
    }
  }

  // 绘制自定义平面
  if (aimer::param::find_int_param("FLASK_SHOW_PLANE") == 1) {
    const Eigen::Vector3d directions[4]{
        {1., 0., 0.}, {0., 1., 0.}, {-1., 0., 0.}, {0., -1., 0.}};
    const cv::Scalar colors[4]{
        {255, 0., 0.}, {0., 255., 0.}, {0., 0., 255.}, {0., 255., 255.}};
    const Eigen::Vector3d corners[4]{
        {0.5, 0.5, 0.}, {0.5, -0.5, 0.}, {-0.5, -0.5, 0.}, {-0.5, 0.5, 0.}};
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector3d center = {
          0., 0., aimer::param::find_double_param("FLASK_PLANE_Z")};
      for (int j = 1; j <= 8; ++j) {
        center += directions[i];
        std::vector<Eigen::Vector3d> pis;
        std::vector<cv::Point2f> pts;
        bool all_z_plus{true};
        for (int k = 0; k < 4; ++k) {
          Eigen::Vector3d pi = center + corners[k];
          Eigen::Vector3d pc = this->converter.pi_to_pc(pi);
          if (pc(2, 0) < 0.) {
            all_z_plus = false;
          }
          pis.push_back(pi);
          pts.push_back(this->converter.pi_to_pu(pi));
        }
        aimer::debug::flask_map << aimer::debug::poses_to_map_lines(
            pis, colors[i], true, aimer::debug::FLASK_MAP_THICKNESS);
        if (all_z_plus)
          aimer::debug::flask_aim
              << aimer::debug::pts_to_map_lines(pts, colors[i], true, 3);
      }
    }
  }

  // 绘制自定义立方体
  if (aimer::param::find_int_param("FLASK_SHOW_CUBE") == 1) {
    double dis{aimer::param::find_double_param("FLASK_CUBE_DISTANCE")};
    const Eigen::Vector3d directions[4]{
        {dis, 0., 0.}, {0., dis, 0.}, {-dis, 0., 0.}, {0., -dis, 0.}};
    const cv::Scalar colors[4]{
        {255, 0., 0.}, {0., 255., 0.}, {0., 0., 255.}, {0., 255., 255.}};
    const Eigen::Vector3d corners[8]{{0.5, 0.5, 0.5},   {0.5, 0.5, -0.5},
                                     {0.5, -0.5, -0.5}, {0.5, -0.5, 0.5},
                                     {-0.5, -0.5, 0.5}, {-0.5, -0.5, -0.5},
                                     {-0.5, 0.5, -0.5}, {-0.5, 0.5, 0.5}};
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector3d center{0., 0.,
                             aimer::param::find_double_param("FLASK_CUBE_Z")};
      center += directions[i];
      for (int k = 0; k < 8; k += 2) {
        std::vector<Eigen::Vector3d> pis;
        std::vector<cv::Point2f> pts;
        bool all_z_plus = true;
        for (int p = k, cnt = 0; cnt < 4; p = (p + 1) % 8, ++cnt) {
          Eigen::Vector3d pi = center + corners[p];
          Eigen::Vector3d pc = this->converter.pi_to_pc(pi);
          if (pc(2, 0) < 0.)
            all_z_plus = false;
          pis.push_back(pi);
          pts.push_back(this->converter.pi_to_pu(pi));
        }
        aimer::debug::flask_map << aimer::debug::poses_to_map_lines(
            pis, colors[i], true, aimer::debug::FLASK_MAP_THICKNESS);
        if (all_z_plus)
          aimer::debug::flask_aim
              << aimer::debug::pts_to_map_lines(pts, colors[i], true, 3);
      }
    }
  }

  // 绘制 被采纳为状态数据的识别结果
  for (int i = 0; i <= aimer::MAX_ENEMY_NUMBER; ++i) {
    for (auto& d : this->enemy_states[i].get_armor_data_ref()) {
      for (int j = 0; j < 4; ++j)
        cv::line(aim_mat, d.info.pus[j], d.info.pus[(j + 1) % 4],
                 {63, 133, 205}, 1);
      for (int j = 0; j < 4; ++j)
        cv::line(aim_mat, d.info.detected.pts[j],
                 d.info.detected.pts[(j + 1) % 4], {255, 255, 0}, 1);

      aimer::math::YpdCoord ypd = aimer::math::xyz_to_ypd(d.info.pos);
      cv::putText(
          aim_mat,
          fmt::format("number: {} id: {}", d.info.detected.number, d.id),
          {(int)d.info.center().x, (int)d.info.center().y},
          cv::FONT_HERSHEY_DUPLEX, 0.66, {0, 150, 0});
      cv::putText(
          aim_mat,
          fmt::format("color ori|fix: {}|{}", d.info.detected.color, d.color),
          {(int)d.info.center().x, (int)d.info.center().y + 25},
          cv::FONT_HERSHEY_DUPLEX, 0.66, {0, 150, 0});
      cv::putText(aim_mat, fmt::format("area: {:.3f}k", d.info.area() / 1000),
                  {(int)d.info.center().x, (int)d.info.center().y + 50},
                  cv::FONT_HERSHEY_DUPLEX, 0.66, {0, 150, 0}, 2);
      cv::putText(aim_mat,
                  fmt::format("ypd {:.1f}|{:.1f}|{:.3f}", ypd.yaw / M_PI * 180.,
                              ypd.pitch / M_PI * 180., ypd.dis),
                  {(int)d.info.center().x, (int)d.info.center().y + 75},
                  cv::FONT_HERSHEY_DUPLEX, 0.66, {0, 150, 0});
      cv::putText(aim_mat,
                  fmt::format("xyz {:.3f}|{:.3f}|{:.3f}", d.info.pos(0, 0),
                              d.info.pos(1, 0), d.info.pos(2, 0)),
                  {(int)d.info.center().x, (int)d.info.center().y + 100},
                  cv::FONT_HERSHEY_DUPLEX, 0.66, {0, 150, 0});
    }
  }

  // 调用 被更新且可能被追踪（如果多枪管）的模型 的绘画函数
  for (auto& number : this->debug_update_list) {
    this->enemy_models[number]->draw_aim(aim_mat);
  }

  // 若被追踪目标不在更新列表里，仍可能被追踪，调用 被追踪目标的模型 的绘画函数
  if (int target = this->target_catcher.get_target();
      this->debug_cmd.shoot != ::ShootMode::IDLE && target != -1 &&
      this->debug_update_list.find(target) == this->debug_update_list.end()) {
    this->enemy_models[target]->draw_aim(aim_mat);
  }

  // 绘制发送的瞄准 (aim) 点，橙色，当且仅当 IDLE 不画
  if (this->debug_cmd.shoot != ::ShootMode::IDLE) {  // not idle
    cv::Point2f exp_pu = this->converter.aim_ypd_to_pu(this->debug_aim.ypd);
    aimer::debug::flask_aim
        << aimer::debug::FlaskPoint(exp_pu, {0, 105, 255}, 10, 4);
    cv::Point2f arrow_end{
        // aim 内部以右转为 yaw_v 正方向
        // 下转为 pitch_v 正方向
        exp_pu.x + (float)(this->debug_aim.ypd_v.yaw / M_PI * 180. * 3.),
        exp_pu.y + (float)(this->debug_aim.ypd_v.pitch / M_PI * 180. * 3.)};
    aimer::debug::draw_arrow(aim_mat, exp_pu, arrow_end, {0, 0, 255}, 2);
    cv::Point2f cmd_pu = this->converter.aim_ypd_to_pu(aimer::math::YpdCoord(
        -this->debug_cmd.yaw / 180. * M_PI,
        -this->debug_cmd.pitch / 180. * M_PI, this->debug_cmd.dist));
    aimer::debug::flask_aim
        << aimer::debug::FlaskPoint(cmd_pu, {0, 255, 255}, 10, 4);
    // 显示适合打击
    if (this->debug_cmd.shoot == ::ShootMode::SHOOT_NOW)
      cv::putText(aim_mat, fmt::format("SHOOT_CMD"), {800, 120},
                  cv::FONT_HERSHEY_TRIPLEX, 3, {0, 0, 255});
  }

  aimer::debug::flask_aim >> aim_mat;
  return aim_mat;
}

cv::Mat EnemyPredictor::draw_map() {
  cv::Mat map_mat = cv::Mat::zeros(
      cv::Size(aimer::debug::FLASK_MAP_WIDTH, aimer::debug::FLASK_MAP_HEIGHT),
      CV_8UC3);
  int width_meter_cnt =
      int(aimer::debug::FLASK_MAP_WIDTH / 2. /
          aimer::param::find_double_param("FLASK_MAP_PIXEL_PER_METER"));
  int height_meter_cnt =
      int(aimer::debug::FLASK_MAP_HEIGHT / 2. /
          aimer::param::find_double_param("FLASK_MAP_PIXEL_PER_METER"));
  {
    for (int i = -width_meter_cnt; i <= width_meter_cnt; ++i)
      for (int j = -height_meter_cnt; j <= height_meter_cnt; ++j)
        aimer::debug::flask_map << aimer::debug::pos_to_map_point(
            {double(i), double(j), 0.}, {255., 255., 255.}, 2, 1);
  }
  aimer::debug::flask_map << aimer::debug::pos_to_map_point(
      {0., 0., 0.}, {255., 255., 255.}, aimer::debug::FLASK_MAP_PT_RADIUS,
      aimer::debug::FLASK_MAP_THICKNESS);
  aimer::debug::flask_map << aimer::debug::poses_to_map_lines(
      {
          {0., 0., 0.},
          this->converter.get_camera_z_i(),
      },
      {127., 127., 127.}, false, aimer::debug::FLASK_MAP_THICKNESS);
  // 绘制相机视野
  {
    // 横 cols, x, width
    // 纵 rows, y, height
    // cv::Size (横, 纵)
    // cv::Mat (纵, 横)
    std::vector<cv::Point2f> pts{
        {0.f, 0.f},
        {0.f, float(this->converter.get_img_ref().rows)},
        {float(this->converter.get_img_ref().cols), 0.f},
        {float(this->converter.get_img_ref().cols),
         float(this->converter.get_img_ref().rows)}};
    std::vector<Eigen::Vector3d> pis = this->converter.pts_to_pis_norm(pts);
    for (auto& pi : pis) {
      aimer::debug::flask_map << aimer::debug::poses_to_map_lines(
          {{0., 0., 0.},
           pi.normalized() * std::min(width_meter_cnt, height_meter_cnt)},
          {0., 127., 0.}, false, aimer::debug::FLASK_MAP_THICKNESS);
    }
  }
  for (int i = 0; i <= aimer::MAX_ENEMY_NUMBER; ++i) {
    const std::vector<aimer::ArmorData>& armors =
        this->enemy_states[i].get_armor_data_ref();
    for (auto& d : armors)
      aimer::debug::flask_map << aimer::debug::pos_to_map_point(
          d.info.pos,
          d.color == static_cast<int>(::EnemyColor::BLUE)
              ? cv::Scalar(255, 0, 0)
              : cv::Scalar(0, 0, 255),
          aimer::debug::FLASK_MAP_PT_RADIUS, aimer::debug::FLASK_MAP_PT_RADIUS);
  }
  aimer::debug::flask_map >> map_mat;
  return map_mat;
}
}  // namespace aimer