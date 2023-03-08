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

#include "../../base/armor_defs.hpp"
#include "../base/defs.hpp"
#include "UltraMultiThread/ObjManager.hpp"
#include "UltraMultiThread/umt.hpp"
#include "common/common.hpp"
#include "core_io/robot.hpp"
#include "./enemy_predictor/enemy_predictor.hpp"
#include "../param/parameter.hpp"

namespace aimer {

namespace umt = ::umt;
using namespace std::chrono_literals;  // like 200ms

class Predictor {
   private:
    aimer::DetectionResult data = {};
    ::RobotCmd send = {};

    std::shared_ptr<::RobotStatus> robot_status;
    std::shared_ptr<::CheckBox> predictor_aim_checkbox;
    std::shared_ptr<::CheckBox> predictor_map_checkbox;
    std::shared_ptr<::base::webview_data::Page> webview_data_page;

    std::unique_ptr<aimer::EnemyPredictor> enemy_predictor;
    int fps, fps_count;
    std::chrono::_V2::system_clock::time_point t1;

    ros::Subscriber detectionResultSub;
    ros::Publisher cmdPub;
    ros::Publisher webviewMapPub;
    ros::Publisher webviewAimPub;

   public:
    void detectionResultCallback(
        const aimer_msgs::DetectionResult::ConstPtr& result) {
        // sensor_msgs/Image => cv::Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            result->img, sensor_msgs::image_encodings::TYPE_8UC3);
        data.img = cv_ptr->image;
        // time => double
        data.timestamp = result->stamp.toSec();
        // geometry_msgs/Quaternion => Eigen Quaternion
        Eigen::Quaterniond _q;
        tf::quaternionMsgToEigen(result->q, _q);
        Eigen::Vector4d vec = _q.coeffs();
        this->data.q = {(float)vec(0), (float)vec(1), (float)vec(2), (float)vec(3)};

        this->onPredict();
    }

    void publishRobotCmd() {
        aimer_msgs::RobotCmd cmd;
        // maybe need some conversions
        cmd.aim_id = this->send.aim_id;
        cmd.car_id = this->send.car_id;
        cmd.detection_info = this->send.detection_info;
        cmd.yaw = this->send.yaw;
        cmd.pitch = this->send.pitch;
        cmd.yaw_v = this->send.yaw_v;
        cmd.pitch_v = this->send.pitch_v;
        cmd.dist = this->send.dist;
        cmd.shoot = (uint8_t)this->send.shoot;
        cmd.period = this->send.period;
        cmd.lrc = this->send.lrc;
        cmd.lock_yaw = this->send.lock_yaw;

        this->cmdPub.publish(cmd);
    }

    void publishWebview(const ros::Publisher& publisher, cv::Mat& img) {
        sensor_msgs::ImagePtr p_img =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        publisher.publish(p_img);
    }

    void onPredict() {
        if (this->robot_status->program_mode == ::ProgramMode::ENERGY_HIT ||
            this->robot_status->program_mode == ::ProgramMode::ENERGY_DISTURB) {
            std::this_thread::sleep_for(200ms);
            // continue;
        }

        if (std::isnan(this->send.yaw) || std::isnan(this->send.pitch) ||
            std::isinf(this->send.yaw) || std::isinf(this->send.pitch)) {
            fmt::print(fmt::fg(fmt::color::red),
                       "=== Predictor output nan of inf, rebuilt it ===\n");
            this->enemy_predictor = nullptr;  // 删除
            this->enemy_predictor = std::make_unique<aimer::EnemyPredictor>();
            std::this_thread::sleep_for(200ms);  // 是否需要 sleep？
            // continue;
        }

        this->send.seq_id++;  // 若没有目标，则 predict 函数内 send 未改动

        publishRobotCmd();

        if (this->predictor_aim_checkbox->checked) {
            this->fps_count++;
            auto t2 = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                    .count() >= 1000) {
                this->fps = this->fps_count;
                this->fps_count = 0;
                this->t1 = t2;
            }
            cv::Mat im2show{this->enemy_predictor->draw_aim(data.img)};
            cv::putText(im2show, fmt::format("fps={}", fps), {10, 25},
                        cv::FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255});
            publishWebview(this->webviewAimPub, im2show);
        }
        if (predictor_map_checkbox->checked) {
            cv::Mat img{enemy_predictor->draw_map()};
            publishWebview(this->webviewMapPub, img);
        }
    }

    void predictor_run(ros::NodeHandle& node) {
        // define publisher & subscriber
        this->detectionResultSub = node.subscribe<aimer_msgs::DetectionResult>(
            "detection_result", 10, &Predictor::detectionResultCallback, this);
        this->cmdPub = node.advertise<aimer_msgs::RobotCmd>("robot_cmd", 10);
        this->webviewMapPub =
            node.advertise<sensor_msgs::Image>("aimer/auto_aim/aim", 10);
        this->webviewAimPub =
            node.advertise<sensor_msgs::Image>("aimer/auto_aim/map", 10);

        // 接收电控数据
        this->robot_status =
            umt::ObjManager<::RobotStatus>::find_or_create("robot_status");
        // 发布器，点击控制指令发送者

        this->predictor_aim_checkbox =
            umt::ObjManager<::CheckBox>::find_or_create(
                "show aimer.auto_aim.aim");

        this->predictor_map_checkbox =
            umt::ObjManager<::CheckBox>::find_or_create(
                "show aimer.auto_aim.map");

        this->webview_data_page =
            umt::ObjManager<::base::webview_data::Page>::find_or_create(
                "aimer.auto_aim.aim");

        while (aimer::param::find_int_obj("PARAM_LOADED") == nullptr) {
            fmt::print(
                fmt::fg(fmt::color::orange),
                "[WARNING] @auto_aim.predictor: 等待 aimer参数全部创建.\n");
            std::this_thread::sleep_for(200ms);
        }
        this->enemy_predictor = std::make_unique<aimer::EnemyPredictor>();
        this->fps = 0;
        this->fps_count = 0;
        this->t1 = std::chrono::system_clock::now();
    }
};
}  // namespace aimer

// PYBIND11_EMBEDDED_MODULE(aimer_auto_aim_predictor, m) {
//     // namespace py = pybind11;
//     // m.def("background_predictor_run", aimer::background_predictor_run);
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "predictor");
    ros::NodeHandle node;
    aimer::Predictor predictor;

    predictor.predictor_run(node);

    ROS_INFO("predictor node is running.");

    ros::spin();
    return 0;
}
