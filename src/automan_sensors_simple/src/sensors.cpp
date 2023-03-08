#include "sensors.hpp"
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "robot_msgs/AutoAimSensorsMsg.h"
#include "robot_msgs/AutoAimSensorsParam.h"

// template<typename T>
CameraIMU::CameraIMU(ros::NodeHandle &ros_node,
                        const std::string &camera_name, 
                        const std::string &camera_cfg,
                        const std::string &sensor_param_file,
                        const std::string &imu_usb_hid,
                        const int sync_period_ms)
  : ros_node(ros_node), camera_name(camera_name), camera_cfg(camera_cfg), 
    sensor_param_file(sensor_param_file), imu_usb_hid(imu_usb_hid), sync_period_ms(sync_period_ms)
{
  
}

// template<typename T>
bool CameraIMU::sync_once(HikVision &camera, ExtImu &imu, int sync_period_ms)
{
  ROS_INFO("=============== sync once ==============\n");
  camera.close();
  require_imu_stop = true;
  imu.stop_trigger();
  while (imu_running)
    ;
  seq_q = std::queue<std::array<double, 4>>();
  camera.open();
  if (!camera.isOpen()) {
    ROS_ERROR("[ERROR]: camera reopen fail!\n");
    return false;
  }
  try {
    imu.periodic_trigger(sync_period_ms);
    ROS_INFO("set trigger period %d\n", (int)sync_period_ms);
  } catch (serial::SerialException& e) {
    ROS_ERROR("[ERROR] {}\n", e.what());
    return false;
  }
  require_imu_stop = false;
  return true;
}

// template<typename T>
void CameraIMU::imu_capture_loop(ExtImu &imu, bool &require_stop, bool & is_ok)
{
  ROS_INFO("entered imu_capture_loop\n");
  ExtImu::sensor_data data{};
  imu_running = true;
  while (!require_stop) {
    if (require_imu_stop) {
      imu.flushInput();
      imu_running = false;
      while (require_imu_stop)
        ;
      imu_running = true;
    }
    try {
      imu.read_sensor(&data);
      // if (data.trigger == 1)
      //   std::cout << "read_imu: " << (int)data.trigger << std::endl;
    } catch (serial::SerialException& e) {
      ROS_ERROR("[ERROR] {}\n", e.what());
      is_ok = false;
      break;
    }

    if (data.trigger == 1) {
      std::unique_lock lock(seq_q_mtx);
      if (!seq_q.empty())
        seq_q.pop();
      seq_q.push({data.q[3], data.q[0], data.q[1], data.q[2]});
    }
  }
}

// template<typename T>
bool CameraIMU::sensors_io_loop()
{
  using namespace std::chrono;
  cv::FileStorage ifs;
  cv::Mat F, C, R_CI;
  if (!ifs.open(sensor_param_file, cv::FileStorage::READ)) {
    ROS_ERROR("[ERROR]: sensor parameter read fail!");
    return false;
  }

  // read sensor param from file and kepp publishing
  ros::Publisher sensor_param_pub = ros_node.advertise<robot_msgs::AutoAimSensorsParam>("camera_info", 100, true);
  robot_msgs::AutoAimSensorsParam sensor_param;
  try {
    ifs["F"] >> F;
    ifs["C"] >> C;
    ifs["R_CI"] >> R_CI;
    if (F.cols != 3 || F.rows != 3) {
      throw std::runtime_error("sensor parameter 'F' invalid format!");
    }
    if (C.cols != 5 || C.rows != 1) {
      throw std::runtime_error("sensor parameter 'C' invalid format!");
    }
    if (R_CI.cols != 3 || R_CI.rows != 3) {
      throw std::runtime_error("sensor parameter 'R_CI' invalid format!");
    }
  } catch (cv::Exception& e) {
    ROS_ERROR("[ERROR]: {}", e.what());
    return false;
  }
  sensor_param.F.data = std::vector<double>(F.reshape(1, 1));
  sensor_param.C.data = std::vector<double>(C.reshape(1, 1));
  sensor_param.R_CI.data = std::vector<double>(R_CI.reshape(1, 1));
  sensor_param_pub.publish(sensor_param);


  // try to open hik camera
  HikVision camera(camera_name.data(), camera_cfg.data());
  camera.EnumDevices();
  camera.open();
  if (!camera.isOpen()) {
    ROS_ERROR("[ERROR]: camera init fail!\n");
    return false;
  }

  // try to open imu
  ExtImu imu;
  for (const auto& port_info : serial::list_ports()) {
    std::cerr << "hardware_id: " << port_info.hardware_id
              << ", port: " << port_info.port << std::endl;
    if (port_info.hardware_id == imu_usb_hid) {
      imu.setPort(port_info.port);
      break;
    }
  }
  imu.open();
  if (!imu.isOpen()) {
    ROS_ERROR("[ERROR]: imu init fail!\n");
    return false;
  }

  ros::Publisher sensor_data_pub = ros_node.advertise<robot_msgs::AutoAimSensorsMsg>("camera_data", 100, true);

  bool imu_require_stop = false;
  bool imu_is_ok = true;
  std::thread imu_capture_thread([&](){
    this->imu_capture_loop(std::ref(imu), std::ref(imu_require_stop), std::ref(imu_is_ok));
  });

  auto t1 = high_resolution_clock::now();
  int fps = 0, fps_count = 0;

  if (this->sync_once(camera, imu, sync_period_ms)) {
    double last_t_cam = 0.;
    double last_t_sys = 0.;

    while (ros::ok() && imu_is_ok) {
      // robot_status_change_recall_loop(camera);
      // 在切换为能量机关模式后更改相机曝光增益

      cv::Mat img;
      double t_sys = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count() /
                     1e3;

      double t_cam;  // ms
      // 在 read 内，timestamp 为 head 缓冲区内的值，该值与 GetImage 同步
      if (!camera.read(img, t_cam)) {  // 暂不读取 timestamp
        ROS_ERROR("Camera read error!\n");
        break;
      }
      // 如果 camera
      // 内缓冲区时间轴存在平移，不影响（该平移的标准差需要远小于读取间隔）

      double after_t_sys =  // (s)
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count() /
          1e3;
      last_t_sys = t_sys;
      last_t_cam = t_cam;

      if (img.rows != 768 || img.cols != 1280)
        cv::resize(img, img, {1280, 768});

      /* publish data */
      {
        std::unique_lock lock(seq_q_mtx);
        if (seq_q.empty()) {
          ROS_ERROR("[ERROR]: Camera received but imu lost!");
          continue;
        }
        ROS_INFO("publish once \n");
        // TODO:
        // data_pub.push({img, seq_q.back(), t_sys});
        robot_msgs::AutoAimSensorsMsg sensor_msg;
        sensor_msg.img = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg());
        std::array<double, 4> q_raw = seq_q.back();
        Eigen::Quaternionf q(q_raw[0], q_raw[1], q_raw[2], q_raw[3]);
        sensor_msg.q = tf2::toMsg(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
        sensor_msg.time_stamp = ros::Time::now();
        sensor_data_pub.publish(sensor_msg);
      }
      sensor_param_pub.publish(sensor_param);
    }
  }

  imu_require_stop = true;
  imu_capture_thread.join();
  return 0;
}