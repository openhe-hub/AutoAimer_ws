//
// Created by xinyang on 2021/3/7.
//
// ver. 22-11-13

#ifndef CORE_IO_ROBOT_HPP
#define CORE_IO_ROBOT_HPP

#include <cstdint>

enum class EnemyColor : uint8_t {
  BLUE = 0,
  RED = 1,
};

enum class ProgramMode : uint8_t {
  MANUAL = 0,
  AUTOAIM = 1,
  ANTITOP = 2,
  ENERGY_HIT = 4,
  ENERGY_DISTURB = 5,
  NOT_RECEIVED = 8,
};

enum class ShootMode : uint8_t {
  TRACKING = 0,
  SHOOT_NOW = 1,
  IDLE = 2,  // 空闲状态
};

struct RobotStatus {
  float bullet_speed;
  float yaw_compensate;
  float pitch_compensate;
  ::EnemyColor enemy_color : 1;  // 这是字节
  uint8_t is_big_energy : 1;
  ::ProgramMode program_mode : 6;
  uint8_t latency_cmd_to_fire;  // (ms) 电控计算得指令到开火延迟
  int last_shoot_aim_id;
  float laser_distance;
  uint8_t lrc;
  RobotStatus()
      : bullet_speed(28.88888f),
        yaw_compensate(1.f),
        pitch_compensate(-1.f),
        enemy_color(::EnemyColor::BLUE),
        is_big_energy(false),
        program_mode(::ProgramMode::NOT_RECEIVED),
        latency_cmd_to_fire(40),
        last_shoot_aim_id(0),
        lrc(0) {}
} __attribute__((packed));

static_assert(sizeof(RobotStatus) != 0);

// 如果拆分协议，恶心点在于电控在不同模式下塞入相同的东西
// 而我在本地如何管理这些不同的包？
// 事实上，视觉所有线程均需要一个状态包来决定自己是否休眠
// 即使不在吊射模式，也需要给吊射线程发包
// 全局 mode？然而糟糕的是，我们无法根据 mode 决定收什么包
// 我们只能先收包

struct RobotCmd {
  uint8_t start = (unsigned)'s';
  uint8_t seq_id = 0;  // 丢包率测试
  int aim_id = 0;
  uint16_t car_id : 4;           // 正在击打的目标
  uint16_t detection_info : 12;  // 识别到的序列(0-8装甲板, 9-10能量机关)
  float yaw = 0.f;
  float pitch = 0.f;
  float yaw_v = 0.f;
  float pitch_v = 0.f;
  float dist = 0.f;
  ::ShootMode shoot = ::ShootMode::TRACKING;
  uint16_t period = 0;  // 2021 版反陀螺的延迟发射要求
  uint8_t lrc = 0;
  bool lock_yaw = 0;
  uint8_t end = (unsigned)'e';
} __attribute__((packed));

// 看看大小
static_assert(sizeof(RobotCmd) != 0);

#endif  // CVRM2021_ROBOT_HPP
