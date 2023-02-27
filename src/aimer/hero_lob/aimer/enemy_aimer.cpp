#include "aimer/hero_lob/aimer/enemy_aimer.hpp"
#include <cmath>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "aimer/base/robot/coord_converter.hpp"
#include "aimer/hero_lob/aimer/calculator.hpp"
#include "aimer/hero_lob/hero_defs.hpp"
#include "aimer/param/parameter.hpp"
#include "common/common.hpp"
#include "core_io/robot.hpp"
using namespace std;
namespace hero {
EnemyAimer::EnemyAimer(
    aimer::CoordConverter* const converter,
    umt::Publisher<RobotCmd>* const robot_cmd,
    std::shared_ptr<RobotStatus>* const robot_status,
    umt::Subscriber<hero::GreenLightDetectionResult>* const subscriber)
    : robot_cmd(robot_cmd),
      robot_status(robot_status),
      subscriber(subscriber),
      converter(converter) {}

void EnemyAimer::aim(const hero::GreenLightDetectionResult& data,
                     ::RobotCmd& send) {
  if (!data.found){
    this->sendornotsend=false;
    return;
  }
  //计算yaw pitch
  float x_avg = 0, y_avg = 0;
  for (int i = 0; i < 4; i++)
    x_avg += data.pts[i].x, y_avg += data.pts[i].y;
  x_avg /= 4, y_avg /= 4;
  auto result = this->converter->pd_to_yp_c(cv::Point2f{x_avg, y_avg});

  //卡尔曼滤波
  this->yaw_filter.update(result.yaw, this->converter->get_img_t(), {0.01, 10.},
                          {0.001});
  this->pitch_filter.update(result.pitch, this->converter->get_img_t(),
                            {0.01, 10.}, {0.001});
  //发送信息
  send.yaw = -aimer::math::rad_to_deg(
                 yaw_filter.predict(this->converter->get_img_t())(0, 0))+aimer::param::find_double_param("HERO_YAW_OFFSET");
  send.pitch = -aimer::math::rad_to_deg(
                   pitch_filter.predict(this->converter->get_img_t())(0, 0));
  //发送信息
  // send.yaw =  aimer::math::rad_to_deg(result.yaw);
  // send.pitch = aimer::math::rad_to_deg(result.pitch);
}

double EnemyAimer::calculator(cv::Point2d tarr, double Ma_he) {
  cv::putText(this->web_push_image, "calculator", {10, 25 + 30 + 30}, 0, 1,
              {0, 0, 255});
  cv::putText(this->web_push_image, "targetx: " + std::to_string(tarr.x),
              {10, 25 + 30 + 30 * 2}, 0, 1, {0, 0, 255});
  cv::putText(this->web_push_image, "targety: " + std::to_string(tarr.y),
              {10, 25 + 30 + 30 * 3}, 0, 1, {0, 0, 255});
  double ans = dandaojiesuan(tarr.x, tarr.y, Ma_he);
  this->send.yaw = 0;
  this->send.pitch = 0;
  cv::putText(this->web_push_image, std::to_string(ans), {10, 25 + 30 + 30 * 4},
              0, 1, {0, 0, 255});
  return ans;
}
bool EnemyAimer::camera_point_to_green_light(
    hero::GreenLightDetectionResult& data) {
  //调试视频输出
  // cv::VideoWriter writer("../1res.avi",
  //                        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 50,
  //                        cv::Size(1280, 768), true);
  // writer << web_image;
  this->aim(data, send);
  //处理web_image
  if (send.yaw > 0)
    cv::putText(this->web_push_image, "right" + std::to_string(send.yaw),
                {10, 25 + 30}, 0, 1, {0, 0, 255});
  else
    cv::putText(this->web_push_image, "left" + std::to_string(send.yaw),
                {10, 25 + 30}, 0, 1, {0, 0, 255});
  if (send.pitch < 0)
    cv::putText(this->web_push_image,
                "                 up" + std::to_string(send.pitch),
                {10, 25 + 30}, 0, 1, {0, 0, 255});
  else
    cv::putText(this->web_push_image,
                "                 down" + std::to_string(send.pitch),
                {10, 25 + 30}, 0, 1, {0, 0, 255});
  cv::line(this->web_push_image, data.pts[0], data.pts[1], {255, 0, 0}, 3);
  cv::line(this->web_push_image, data.pts[1], data.pts[2], {255, 0, 0}, 3);
  cv::line(this->web_push_image, data.pts[2], data.pts[3], {255, 0, 0}, 3);
  cv::line(this->web_push_image, data.pts[3], data.pts[0], {255, 0, 0}, 3);
  // return false;
  const double end_val = 0.01;  //当与绿灯距离小于此值时结束相机对准绿灯
  if (send.yaw < end_val && send.pitch < end_val)
    return true;
  }
  else{return false;}
}
void EnemyAimer::shoot(double angle, double yaw, double pitch) {
  cv::putText(web_push_image, "ready to shoot", {10, 25 + 30}, 0, 1,
              {0, 0, 255});
  cv::putText(this->web_push_image,
              "target: " + std::to_string(aimer::math::rad_to_deg(yaw)) + " " +
                  std::to_string(pitch),
              {10, 25 + 30 * 4}, 0, 1, {0, 0, 255});
  this->send.shoot = ShootMode::SHOOT_NOW;
  this->send.yaw = 0;
  this->send.pitch = 0;
  return;
}
bool EnemyAimer::find_pitch_target(double& max_distance) {
  cv::putText(this->web_push_image, "up_scan", {10, 25 + 30}, 0,
              1, {0, 0, 255});
  double laser_d = this->converter->get_robot_status_ref().laser_distance;
  const double max_d = 12;  //激光测距超过此距离时结束上扫
  if (laser_d > max_d)
    return true;
  if (laser_d > max_distance)
    max_distance = laser_d;

  const double delta_pitch = 0.008;
  this->send.pitch = delta_pitch;
  this->send.yaw = 0;
  return false;
}
bool EnemyAimer::find_pitch_target2(double& max_distance) {
   cv::putText(this->web_push_image, "down_scan", {10, 25 + 30}, 0,
              1, {0, 0, 255});
  double laser_d = this->converter->get_robot_status_ref().laser_distance;
  //激光测距略小于接近max_distance时结束下扫
  if (laser_d <= max_distance+0.2)
    return true;
  const double delta_pitch = -0.001;
  this->send.pitch = delta_pitch;
  this->send.yaw = 0;
  return false;
}
cv::Point2d EnemyAimer::laser(hero::GreenLightDetectionResult& data,
                              cv::Point2d* target_yaw_pitch) {
  double laser_d = this->converter->get_robot_status_ref().laser_distance;
  double pitch_angle = converter->get_camera_z_i_pitch();
  target_yaw_pitch->x = converter->get_camera_z_i_yaw();
  cv::putText(this->web_push_image, "laser", {10, 25 + 30}, 0, 1, {0, 0, 255});
  this->send.yaw = 0;
  this->send.pitch = 0;
  return cv::Point2d(laser_d * cos(pitch_angle) + 0.09023 * cos(pitch_angle) -
                         0.14774 * sin(pitch_angle),
                     0.09023 * sin(pitch_angle) + laser_d * sin(pitch_angle) +
                         0.14774 * cos(pitch_angle));
}
bool EnemyAimer::move_to_target_yawpitch(
    double yaw,
    double pitch,
    hero::GreenLightDetectionResult& data) {
  // this->send.yaw =
  //     aimer::math::rad_to_deg(yaw - converter->get_camera_z_i_yaw());  //要换算
  this->send.pitch = (pitch - aimer::math::rad_to_deg(
                                  converter->get_camera_z_i_pitch()));  //要换算
  cv::putText(this->web_push_image, "move_to_target_yawpitch", {10, 25 + 30}, 0,
              1, {0, 0, 255});
  cv::putText(this->web_push_image,
              "send: " + std::to_string(this->send.yaw) + " " +
                  std::to_string(this->send.pitch),
              {10, 25 + 30 * 2}, 0, 1, {0, 0, 255});
  cv::putText(this->web_push_image,
              "now: " +
                  std::to_string(aimer::math::rad_to_deg(
                      converter->get_camera_z_i_yaw())) +
                  " " +
                  std::to_string(aimer::math::rad_to_deg(
                      converter->get_camera_z_i_pitch())),
              {10, 25 + 30 * 3}, 0, 1, {0, 0, 255});
  cv::putText(this->web_push_image,
              "target: " + std::to_string(aimer::math::rad_to_deg(yaw)) + " " +
                  std::to_string(pitch),
              {10, 25 + 30 * 4}, 0, 1, {0, 0, 255});

  // return false;
  const double end_val = 0.01;  //与目标角度小于此值时已到达目标角度
  if (send.yaw < end_val && send.pitch < end_val)this->cnt_time++;
  if (this->cnt_time>50){
    this->cnt_time=0;
    return true;}
  else{
    return false;}
}
void EnemyAimer::sleep() {
    this->send.yaw = 0;
  this->send.pitch = 0;
  cv::putText(this->web_push_image, "sleeping", {10, 25 + 30 * 3}, 0, 1,
              {0, 0, 255});
}
}  // namespace hero
