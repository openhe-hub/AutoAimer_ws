#include <chrono>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <thread>

#include <fmt/color.h>
#include <fmt/format.h>
#include <pybind11/numpy.h>

#include "aimer/base/robot/coord_converter.hpp"
#include "aimer/hero_lob/aimer/enemy_aimer.hpp"
#include "aimer/hero_lob/hero_defs.hpp"
#include "aimer/param//parameter.hpp"
#include "common/common.hpp"
#include "core_io/robot.hpp"

namespace hero {
enum class HeroLobStatus {
  INIT,
  CAMERA_TO_LIGHT,
  FIND_PITCH_TARGET1,
  FIND_PITCH_TARGET2,
  BULLET_SIMUL,
  PITCH_UP,
  SHOOT_CONFIRM,
  SLEEP
};
void aimer_run() {
  using namespace std::chrono_literals;
  // 订阅器，接收来自 detector 线程的识别数据
  auto subscriber =
      umt::Subscriber<GreenLightDetectionResult>("green_light_detections");
  // 接收电控数据
  auto robot_status =
      umt::ObjManager<::RobotStatus>::find_or_create("robot_status");
  // 发布器，点击控制指令发送者
  auto robot_cmd = umt::Publisher<::RobotCmd>("robot_cmd");
  //坐标转换器
  aimer::CoordConverter converter;
  //接收信息发送信息
  hero::EnemyAimer enemy_aimer{&converter, &robot_cmd, &robot_status,
                               &subscriber};
  hero::GreenLightDetectionResult data = {};
  //前端可视化
  auto predictor_aim_checkbox =
      umt::ObjManager<::CheckBox>::find_or_create("hero_lob show aimer_result");
  auto webview_predictor_aim = umt::Publisher<cv::Mat>("hero_lob_aimer_result");
  predictor_aim_checkbox->checked = 1;
  int fps = 0, fps_count = 0;
  auto t1 = std::chrono::system_clock::now();
  //状态切换变量
  HeroLobStatus status = HeroLobStatus::SLEEP;
  bool change = false;
  cv::Point2d target(0, 0);
  double shot_angle = 0;  //它是角度，不是弧度
  cv::Point2d target_yaw_pitch(0, 0);
  double max_distance = 0;
  while (aimer::param::find_int_obj("PARAM_LOADED") == nullptr) {
    std::cerr << "Wating for param obj all created.\n";
    std::this_thread::sleep_for(200ms);
  }
  while (true) {
    try {
      change = false;
      enemy_aimer.sendornotsend=true;
      enemy_aimer.send.lock_yaw=false;
      data = subscriber.pop();

      // web_push_image为推送到网页端的图片，由各步骤的函数处理
      enemy_aimer.web_push_image = data.img.clone();
      //更新坐标转换器
      converter.update(
          aimer::CoordConverter::UpdateData{data.img, data.q, data.timestamp});

      //计算fps，标注在网页上
      if (predictor_aim_checkbox->checked) {
        fps_count++;
        auto t2 = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                .count() >= 1000) {
          fps = fps_count;
          fps_count = 0;
          t1 = t2;
        }
        cv::putText(enemy_aimer.web_push_image, fmt::format("fps={}", fps),
                    {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255});
        cv::putText(
            enemy_aimer.web_push_image,
            "laser_distance: " + std::to_string(robot_status->laser_distance),
            {10, 25 + 30 * 7}, 0, 1, {0, 0, 255});
        cv::putText(enemy_aimer.web_push_image,
                    "now_yaw: " + std::to_string(aimer::math::rad_to_deg(
                                      converter.get_camera_z_i_yaw())),
                    {10, 25 + 30 * 8}, 0, 1, {0, 0, 255});
        cv::putText(enemy_aimer.web_push_image,
                    "now_pitch: " + std::to_string(aimer::math::rad_to_deg(
                                        converter.get_camera_z_i_pitch())),
                    {10, 25 + 30 * 9}, 0, 1, {0, 0, 255});
        cv::putText(enemy_aimer.web_push_image,
                    "bullet_speed: " + std::to_string(robot_status->bullet_speed),
                    {10, 25 + 30 * 10}, 0, 1, {0, 0, 255});
        cv::putText(enemy_aimer.web_push_image, "max_distance: "+std::to_string(max_distance), {10, 25 + 30*11}, 0,
              1, {0, 0, 255});
      }
      if (aimer::param::find_int_param("HERO_LOB_RESTART") == 1)
        status = HeroLobStatus::INIT;
      switch (status) {
        case HeroLobStatus::INIT:  //初始化
          status = HeroLobStatus(
              aimer::param::find_int_param("HERO_LOB_START_STATUS"));
          // status = HeroLobStatus::CAMERA_TO_LIGHT;
          change = false;
          target = cv::Point2d(0, 0);
          target_yaw_pitch = cv::Point2d(0, 0);
          shot_angle = 0;
          max_distance = 0;
          enemy_aimer.send.yaw=0;
          enemy_aimer.send.pitch=0;
          break;
        case HeroLobStatus::CAMERA_TO_LIGHT:  // 1.先让相机对准
          change = enemy_aimer.camera_point_to_green_light(data);
          if (change)
            status = HeroLobStatus::FIND_PITCH_TARGET1;
          break;
        case HeroLobStatus::
            FIND_PITCH_TARGET1:  // 2.在pitch轴向上移动.测得两装甲板交界处的距离
          change = enemy_aimer.find_pitch_target(max_distance);
          if (change)
            status = HeroLobStatus::FIND_PITCH_TARGET2;
          enemy_aimer.send.lock_yaw=true;
          break;
        case HeroLobStatus::
            FIND_PITCH_TARGET2:  // 2.5. 在pitch轴极慢向下移动.对准打击点
          change = enemy_aimer.find_pitch_target2(max_distance);
          if (change)
            status = HeroLobStatus::BULLET_SIMUL;
          enemy_aimer.send.lock_yaw=true;
          break;
        case HeroLobStatus::
            BULLET_SIMUL:  // 3.激光传来数据，解算好坐标，坐标发给弹道解算程序，算出来个角度
          target = enemy_aimer.laser(data, &target_yaw_pitch);
          if(robot_status->bullet_speed==0)
            robot_status->bullet_speed=aimer::param::find_double_param("HERO_AVERAGE_BULLET_SPEED");
          shot_angle =
              enemy_aimer.calculator(target, robot_status->bullet_speed);
          target_yaw_pitch.y = shot_angle;
          status = HeroLobStatus::PITCH_UP;
          enemy_aimer.send.lock_yaw=true;
          break;
        case HeroLobStatus::PITCH_UP:  // 4.移动至角度
          change = enemy_aimer.move_to_target_yawpitch(
              target_yaw_pitch.x, target_yaw_pitch.y-3, data);
          if (change)
            status = HeroLobStatus::SHOOT_CONFIRM;
          enemy_aimer.send.lock_yaw=true;
          break;
        case HeroLobStatus::SHOOT_CONFIRM:  // 5.抬枪口到这个角度并射击
          enemy_aimer.shoot(shot_angle,target_yaw_pitch.x, target_yaw_pitch.y);
          // status = HeroLobStatus::SLEEP;
          break;
          enemy_aimer.send.lock_yaw=true;
        case HeroLobStatus::SLEEP:
          enemy_aimer.sleep();
      }
      if(enemy_aimer.sendornotsend)
        robot_cmd.push(enemy_aimer.send);
      //如果网页复选框勾选，则将图像推送过去
      if (predictor_aim_checkbox->checked)
        webview_predictor_aim.push(enemy_aimer.web_push_image);
    } catch (umt::MessageError& e) {
      fmt::print(fmt::fg(fmt::color::orange), "[WARNING] 'sensors_data' {}\n",
                 e.what());
      std::this_thread::sleep_for(500ms);
    }
  }
}
void background_aimer_run() {
  std::cerr << "=========================background_aimer_run=============="
               "================"
            << std::endl;
  // std::thread([]() { pd::param::background_parameter_run(); }).detach();
  std::thread([]() { hero::aimer_run(); }).detach();
}
}  // namespace hero

PYBIND11_EMBEDDED_MODULE(Heroaim2, m) {
  // namespace py = pybind11;
  m.def("background_aimer_run", hero::background_aimer_run);
}
