#include <chrono>
#include <thread>

#include <aimer_msgs/DetectionResult.h>
#include <aimer_msgs/RobotCmd.h>
#include <aimer_msgs/RobotStatus.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/opencv.hpp"

#include <fmt/color.h>
#include <fmt/format.h>
#include <pybind11/numpy.h>

#include "UltraMultiThread/ObjManager.hpp"
#include "UltraMultiThread/umt.hpp"
#include "../base/defs.hpp"
#include "../../base/armor_defs.hpp"
#include "common/common.hpp"
#include "enemy_predictor/enemy_predictor.hpp"
#include "core_io/robot.hpp"

namespace aimer {

namespace umt = ::umt;
using namespace std::chrono_literals;  // like 200ms

aimer::DetectionResult data = {};
::RobotCmd send = {};


void detectionResultCallback(const aimer_msgs::DetectionResult::ConstPtr& result);
void publishRobotCmd(const ros::Publisher& publisher, ::RobotCmd robotCmd);
void publishWebview(const ros::Publisher& publisher, cv::Mat& img);
void onPredict();
void predictor_run(const ros::Publisher& cmdPub,
                   const ros::Publisher& webviewMapPub,
                   const ros::Publisher& webviewAimPub);


void detectionResultCallback(
    const aimer_msgs::DetectionResult::ConstPtr& result) {
    // sensor_msgs/Image => cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        result->img, sensor_msgs::image_encodings::TYPE_8UC3);
    data.img = cv_ptr->image;
    // time => double
    data.timestamp = result->stamp.toSec();
    // geometry_msgs/Quaternion => Eigen Quaternion
    tf::quaternionMsgToEigen(result->q, data.q);
    
    onPredict();
}

void publishRobotCmd(const ros::Publisher& publisher, ::RobotCmd robotCmd) {
    aimer_msgs::RobotCmd cmd;
    // maybe need some conversions
    cmd.aim_id = robotCmd.aim_id;
    cmd.car_id = robotCmd.car_id;
    cmd.detection_info = robotCmd.detection_info;
    cmd.yaw = robotCmd.yaw;
    cmd.pitch = robotCmd.pitch;
    cmd.yaw_v = robotCmd.yaw_v;
    cmd.pitch_v = robotCmd.pitch_v;
    cmd.dist = robotCmd.dist;
    cmd.shoot = robotCmd.shoot;
    cmd.period = robotCmd.period;
    cmd.lrc = robotCmd.lrc;
    cmd.lock_yaw = robotCmd.lock_yaw;

    publisher.publish(cmd);
}

void publishWebview(const ros::Publisher& publisher, cv::Mat& img) {
    sensor_msgs::ImagePtr img =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    publisher.publish(*img);
}

void onPredict() {
    if (robot_status->program_mode == ::ProgramMode::ENERGY_HIT ||
        robot_status->program_mode == ::ProgramMode::ENERGY_DISTURB) {
        std::this_thread::sleep_for(200ms);
        continue;
    }
    send = enemy_predictor->predict(data);

    if (std::isnan(send.yaw) || std::isnan(send.pitch) ||
        std::isinf(send.yaw) || std::isinf(send.pitch)) {
        fmt::print(fmt::fg(fmt::color::red),
                   "====================Predictor output nan of inf, rebuilt
                       it == ==
                       ==
                   = "
                     "=============\n");
        enemy_predictor = nullptr;  // 删除
        enemy_predictor = std::make_unique<aimer::EnemyPredictor>();
        std::this_thread::sleep_for(200ms);  // 是否需要 sleep？
        continue;
    }

    send.seq_id++;  // 若没有目标，则 predict 函数内 send 未改动

    publishRobotCmd(cmdPub, send);

    if (predictor_aim_checkbox->checked) {
        fps_count++;
        auto t2 = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                .count() >= 1000) {
            fps = fps_count;
            fps_count = 0;
            t1 = t2;
        }
        cv::Mat im2show{enemy_predictor->draw_aim(data.img)};
        cv::putText(im2show, fmt::format("fps={}", fps), {10, 25},
                    cv::FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255});
        publishWebview(webviewAimPub, im2show);
    }
    if (predictor_map_checkbox->checked) {
        cv::Mat img{enemy_predictor->draw_map()};
        publishWebview(webviewMapPub, img);
    }
}

void predictor_run(const ros::Publisher& cmdPub,
                   const ros::Publisher& webviewMapPub,
                   const ros::Publisher& webviewAimPub) {
    // 接收电控数据
    auto robot_status =
        umt::ObjManager<::RobotStatus>::find_or_create("robot_status");
    // 发布器，点击控制指令发送者

    auto predictor_aim_checkbox =
        umt::ObjManager<::CheckBox>::find_or_create("show
        aimer.auto_aim.aim");

    auto predictor_map_checkbox =
        umt::ObjManager<::CheckBox>::find_or_create("show
        aimer.auto_aim.map");

    auto webview_data_page =
        umt::ObjManager<::base::webview_data::Page>::find_or_create(
            "aimer.auto_aim.aim");

    while (aimer::param::find_int_obj("PARAM_LOADED") == nullptr) {
      fmt::print(fmt::fg(fmt::color::orange),
                 "[WARNING] @auto_aim.predictor: 等待 aimer
                 参数全部创建.\n");
      std::this_thread::sleep_for(200ms);
    }
    auto enemy_predictor = std::make_unique<aimer::EnemyPredictor>();
    int fps = 0, fps_count = 0;
    auto t1 = std::chrono::system_clock::now();

    while (true) {
        try {
            // MANUAL: 计算但不动枪口且不发射（可发信号来给 UI 反馈）
            // ENERGY相关: 停止计算进程，给能量机关计算
            if (robot_status->program_mode == ::ProgramMode::ENERGY_HIT ||
                robot_status->program_mode == ::ProgramMode::ENERGY_DISTURB) {
                std::this_thread::sleep_for(200ms);
                continue;
            }
            // data = subscriber.pop();
            send = enemy_predictor->predict(data);

            if (std::isnan(send.yaw) || std::isnan(send.pitch) ||
                std::isinf(send.yaw) || std::isinf(send.pitch)) {
                fmt::print(
                    fmt::fg(fmt::color::red),
                    "====================Predictor output nan of inf, rebuilt
                        it == ==
                        ==
                    = "
                      "=============\n");
                enemy_predictor = nullptr;  // 删除
                enemy_predictor = std::make_unique<aimer::EnemyPredictor>();
                std::this_thread::sleep_for(200ms);  // 是否需要 sleep？
                continue;
            }

            send.seq_id++;  // 若没有目标，则 predict 函数内 send 未改动

            publishRobotCmd(cmdPub, send);

            if (predictor_aim_checkbox->checked) {
                fps_count++;
                auto t2 = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
                                                                          t1)
                        .count() >= 1000) {
                    fps = fps_count;
                    fps_count = 0;
                    t1 = t2;
                }
                cv::Mat im2show{enemy_predictor->draw_aim(data.img)};
                cv::putText(im2show, fmt::format("fps={}", fps), {10, 25},
                            cv::FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255});
                publishWebview(webviewAimPub, im2show);
            }
            if (predictor_map_checkbox->checked) {
                cv::Mat img{enemy_predictor->draw_map()};
                publishWebview(webviewMapPub, img);
            }
        } catch (umt::MessageError& e) {
            fmt::print(
                fmt::fg(fmt::color::orange), "[WARNING] 'detection_data'
                                             {}\n ",
                                             e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

// void background_predictor_run() {
//     std::cerr
//         << "=========================background_predictor_run=============="
//            "================"
//         << std::endl;
//     std::thread([]() { aimer::predictor_run(); }).detach();
// }
// }  // namespace aimer

// PYBIND11_EMBEDDED_MODULE(aimer_auto_aim_predictor, m) {
//     // namespace py = pybind11;
//     m.def("background_predictor_run", aimer::background_predictor_run);
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "predictor");
    ros::NodeHandle node;

    ros::Subscriber detectionResultSub =
        node.subscribe<aimer_msgs::DetectionResult>(
            "detection_result", 10, aimer::detectionResultCallback);
    ros::Publisher robotCmdPub =
        node.advertise<aimer_msgs::RobotCmd>("robot_cmd", 10);
    ros::Publisher webview_predictor_aim =
        node.advertise<sensor_msgs::Image>("aimer.auto_aim.aim", 10);
    ros::Publisher webview_predictor_map =
        node.advertise<sensor_msgs::Image>("aimer.auto_aim.map", 10);

    predictor_run();

    ros::spin();
    return 0;
}
