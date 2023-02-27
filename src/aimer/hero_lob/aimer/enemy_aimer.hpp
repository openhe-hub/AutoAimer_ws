#ifndef _AIMER_ENEMY_AIMER_HPP_
#define _AIMER_ENEMY_AIMER_HPP_

#include "aimer/base/robot/coord_converter.hpp"
#include "aimer/hero_lob/hero_defs.hpp"
#include "common/common.hpp"
namespace hero {
class EnemyAimer {
public:
  EnemyAimer(
      aimer::CoordConverter *const converter,
      umt::Publisher<RobotCmd> *const robot_cmd,
      std::shared_ptr<RobotStatus> *const robot_status,
      umt::Subscriber<hero::GreenLightDetectionResult> *const subscriber);
  /*
  《使得相机和绿灯对准》中用到的函数
  自瞄，根据识别结果计算发送电控的数据
  输入detector传来的data，和要发送的send的引用
  使得相机中心对准绿灯
  */
  void aim(const hero::GreenLightDetectionResult &data, ::RobotCmd &send);

  /*
  使得相机和绿灯对准
  其中web_image为推送给网页的图像
  若已经对准则返回真，否则返回假
  */
  bool camera_point_to_green_light(hero::GreenLightDetectionResult &data);

  /*
  弹道解算
  输入目标点，弹道马赫数
  输出最佳角度
  */
  double calculator(cv::Point2d tarr, double Ma_he);

  /*
  朝目标角度射击
  输入角度
  输出这一发子弹的弹速
  */
  void shoot(double angle,double yaw,double pitch);

  /*
  激光传来数据，解算好坐标
  输出目标x，y坐标
  */
  cv::Point2d laser(hero::GreenLightDetectionResult &data,
                    cv::Point2d *target_yaw_pitch);

  /*
  在pitch轴向上移动.测得两装甲板交界处的距离
  若已完成，返回真，否则返回假
  */
  bool find_pitch_target(double &max_distance);
  /*
  在pitch轴极慢向下移动.对准打击点
  若已完成，返回真，否则返回假
  */
  bool find_pitch_target2(double &max_distance);

  /*
 输入目标yaw，pitch
 若已完成，返回真，否则返回假
 */
  bool move_to_target_yawpitch(double yaw, double pitch,
                               hero::GreenLightDetectionResult &data);
  void sleep();
  ::RobotCmd send;
  umt::Publisher<RobotCmd> *const robot_cmd;
  std::shared_ptr<RobotStatus> *const robot_status;
  umt::Subscriber<hero::GreenLightDetectionResult> *const subscriber;
  aimer::CoordConverter *const converter;
  cv::Mat web_push_image;
  bool sendornotsend;
  int cnt_time=0;

private:
  aimer::SingleFilter<2> yaw_filter;   // yaw卡尔曼滤波器
  aimer::SingleFilter<2> pitch_filter; // pitch卡尔曼滤波器
};
} // namespace hero

#endif