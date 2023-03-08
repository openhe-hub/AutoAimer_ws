//
// Created by xinyang on 2021/3/6.
//

// Modified by Harry-hhj on 2021/05/04

#ifndef CORE_IO_SENSORS_HPP
#define CORE_IO_SENSORS_HPP

#include <opencv2/core.hpp>
#include <mutex>
#include <queue>
#include <vector>
#include <string>

#include <ExtImu.hpp>
#include <hikvision.h>
#include <mindvision.hpp>
#include <ros/ros.h>

struct SensorsData {
  cv::Mat im;
  std::array<double, 4> q;
  double timestamp;  // ms
};

struct SensorParam {
  cv::Mat F;
  cv::Mat C;
  cv::Mat R_CI;
};

// template<typename T>
class CameraIMU
{
private:
  ros::NodeHandle &ros_node;

  const std::string &camera_name;
  const std::string &camera_cfg;
  const std::string &sensor_param_file;
  const std::string &imu_usb_hid;
  const int sync_period_ms;


  std::mutex seq_q_mtx;
  std::queue<std::array<double, 4>> seq_q;
  bool imu_running;
  bool require_imu_stop;

  cv::Mat F, C, R_CI;

  
  // bool sync_once(T &camera, ExtImu& imu, int sync_period_ms);
  bool sync_once(HikVision &camera, ExtImu& imu, int sync_period_ms);
  
public:
  explicit CameraIMU(ros::NodeHandle &ros_node,
                     const std::string &camera_name, 
                     const std::string &camera_cfg,
                     const std::string &sensor_param_file,
                     const std::string &imu_usb_hid,
                     const int sync_period_ms = 10);
  void imu_capture_loop(ExtImu &imu, bool& require_stop, bool & is_ok);
  bool sensors_io_loop();
};

#endif /* CORE_IO_SENSORS_HPP */
