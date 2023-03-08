//
// Created by xinyang on 2021/3/6.
//

// Modified by Harry-hhj on 2021/05/04

#include "sensors.hpp"

#include <chrono>
#include <queue>
#include <string>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <serial/serial.h>
#include <iostream>

#include <ros/ros.h>

#include "ExtImu.hpp"

#ifdef WITH_MINDVISION
#include "MindVision.hpp"
#endif
#ifdef WITH_HIKVISION
#include "hikvision.h"
#endif

using namespace std::chrono;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensors");
  ros::NodeHandle nh("~");

  std::string camera_name, camera_cfg, sensor_param_file, imu_usb_hid;
  int sync_period_ms;

  bool read_param_success;
  read_param_success = nh.getParam("camera_name", camera_name);
  if (!read_param_success)
    ROS_ERROR("Parameter camera_name not found\n");
  ROS_INFO("Load parameter camera_name: %s\n", camera_name.c_str());

  read_param_success = nh.getParam("camera_cfg", camera_cfg);
  if (!read_param_success)
    ROS_ERROR("Parameter camera_cfg not found\n");
  ROS_INFO("Load parameter camera_cfg: %s\n", camera_cfg.c_str());
  
  read_param_success = nh.getParam("sensor_param_file", sensor_param_file);
  if (!read_param_success)
    ROS_ERROR("Parameter sensor_param_file not found\n");
  ROS_INFO("Load parameter sensor_param_file: %s\n", sensor_param_file.c_str());
  
  read_param_success = nh.getParam("imu_usb_hid", imu_usb_hid);
  if (!read_param_success)
    ROS_ERROR("Parameter imu_usb_hid not found\n");
  ROS_INFO("Load parameter imu_usb_hid: %s\n", imu_usb_hid.c_str());

  read_param_success = nh.getParam("sync_period_ms", sync_period_ms);
  if (!read_param_success)
  {
    sync_period_ms = 10;
    ROS_WARN("Parameter sync_period_ms not found, use default sync_period_ms = %d\n", sync_period_ms);
  }
    
  ROS_INFO("Load parameter sync_period_ms: %d\n", sync_period_ms);

  CameraIMU camera_imu(std::ref(nh), camera_name, camera_cfg, sensor_param_file, imu_usb_hid, sync_period_ms);
  camera_imu.sensors_io_loop();


  return 0;
}