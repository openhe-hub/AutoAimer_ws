//
// Created by sjturm on 2020/7/13.
//
// Edited by shenyibo on 2022/6/13. its for opencv

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <aimer_msgs/DetectionResult.h>
#include <aimer_msgs/SensorsData.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include <fmt/color.h>
#include <fmt/format.h>
#include <pybind11/numpy.h>
#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <UltraMultiThread/umt.hpp>
#include "../../base/armor_defs.hpp"
#include "../base/defs.hpp"
#include "TRTModule.hpp"
#include "base/debug/print.hpp"
#include "common/common.hpp"
#include "core_io/robot.hpp"
#include "core_io/sensors.hpp"

#define DEAD_GRAY_ARMOR 0
#define LIVED_GRAY_ARMOR 1

bool state = false;
bool lockTargetState = false;
int lockTargetCnt = 0;
int lockTargetIdx = -1;
bool loseTargetFlag = true;
using namespace std::chrono;
namespace py = pybind11;

float thres_partiel_armor[10] = {0.5f, 0.6f,  0.6f,  0.6f,  0.6f,
                                 0.6f, 0.35f, 0.35f, 0.35f, 0.0f};
float thres_partiel_energy[10] = {0.35f, 0.4f, 0.4f, 0.0f, 0.0f,
                                  0.0f,  0.0f, 0.0f, 0.0f, 0.0f};
const Attributes attributes_armor =
    (struct Attributes){4, 4, 9, thres_partiel_armor, true};
const Attributes attributes_energy =
    (struct Attributes){5, 2, 3, thres_partiel_energy, true};

void publishDetectionResult(const ros::Publisher& publisher,
                            aimer::DetectionResult result);
void sensorSubCallback(const aimer_msgs::SensorsData::ConstPtr& data);

void draw_boxes(cv::Mat& src, const std::vector<bbox_t>& results) {
    cv::Scalar color_map[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
    for (const auto& box : results) {
        const auto& color = color_map[box.color_id];
        cv::line(src, box.pts[0], box.pts[1], color, 1);
        cv::line(src, box.pts[1], box.pts[2], color, 1);
        cv::line(src, box.pts[2], box.pts[3], color, 1);
        cv::line(src, box.pts[3], box.pts[0], color, 1);
        cv::putText(src, std::to_string(box.tag_id), box.pts[0], 0, 1, color);
        cv::putText(src, std::to_string(box.confidence).substr(0, 4),
                    box.pts[2], 0, 1, color);
        cv::putText(src, "c" + std::to_string(box.confidence_cls).substr(0, 4),
                    box.pts[3], 0, 1, color);
    }
}

bool cmp_armor(aimer::DetectedArmor a1, aimer::DetectedArmor a2) {
    auto l1 = sqrt(pow((a1.pts[0].x - a1.pts[3].x), 2) +
                   pow((a1.pts[0].y - a1.pts[3].y), 2));
    auto h1 = sqrt(pow((a1.pts[0].x - a1.pts[1].x), 2) +
                   pow((a1.pts[0].y - a1.pts[1].y), 2));
    auto l2 = sqrt(pow((a2.pts[0].x - a2.pts[3].x), 2) +
                   pow((a2.pts[0].y - a2.pts[3].y), 2));
    auto h2 = sqrt(pow((a2.pts[0].x - a2.pts[1].x), 2) +
                   pow((a2.pts[0].y - a2.pts[1].y), 2));
    return (l1 * h1) > (l2 * h2);
}

// 根据离装甲板中心距离确定
bool cmp_armor_center(aimer::DetectedArmor a1, aimer::DetectedArmor a2) {
    cv::Point2d center = {640, 384};
    cv::Point2d center1 = {
        (a1.pts[0].x + a1.pts[1].x + a1.pts[2].x + a1.pts[3].x) / 4.f,
        (a1.pts[0].y + a1.pts[1].y + a1.pts[2].y + a1.pts[3].y) / 4.f};
    cv::Point2d center2 = {
        (a2.pts[0].x + a2.pts[1].x + a2.pts[2].x + a2.pts[3].x) / 4.f,
        (a2.pts[0].y + a2.pts[1].y + a2.pts[2].y + a2.pts[3].y) / 4.f};
    auto dist1 =
        sqrt(pow(center1.x - center.x, 2) + pow(center1.y - center.y, 2));
    auto dist2 =
        sqrt(pow(center2.x - center.x, 2) + pow(center2.y - center.y, 2));
    if (dist1 < dist2) {
        return true;
    } else
        return false;
}

cv::Point2d getBoxCenter(bbox_t b) {
    auto k1 = (b.pts[2].y - b.pts[0].y) / (b.pts[2].x - b.pts[0].x);
    auto b1 = k1 * b.pts[0].x + b.pts[0].y;
    auto k2 = (b.pts[3].y - b.pts[1].y) / (b.pts[3].x - b.pts[1].x);
    auto b2 = k2 * b.pts[3].x + b.pts[3].y;

    double x = (b2 - b1) / (k1 - k2);
    double y = k1 * x + b1;

    return {x, y};
}

bool get_Dist(bbox_t res1, bbox_t res2) {
    double dist = 0;
    double thresh = 100;
    cv::Point2f center_res1, center_res2;

    thresh = 0.5 * sqrt(pow((res1.pts[0].x - res2.pts[3].x), 2) +
                        pow((res1.pts[0].y - res2.pts[3].y), 2));
    center_res1 = getBoxCenter(res1);
    center_res2 = getBoxCenter(res2);
    dist = sqrt(pow((center_res1.x - center_res2.x), 2) +
                pow((center_res1.x - center_res2.x), 2));

    if (dist < thresh)
        return true;
    else
        return false;
}

extern cv::Point2d getCenter(aimer::DetectedArmor a) {
    auto k1 = (a.pts[2].y - a.pts[0].y) / (a.pts[2].x - a.pts[0].x);
    auto b1 = k1 * a.pts[0].x + a.pts[0].y;
    auto k2 = (a.pts[3].y - a.pts[1].y) / (a.pts[3].x - a.pts[1].x);
    ;
    auto b2 = k2 * a.pts[3].x + a.pts[3].y;

    double x = (b2 - b1) / (k1 - k2);
    double y = k1 * x + b1;

    return {x, y};
}
bool getTargetArmor(aimer::DetectedArmor a,
                    aimer::DetectedArmor l_a,
                    double& thresh) {
    thresh = sqrt(pow((a.pts[0].x - a.pts[3].x), 2) +
                  pow((a.pts[0].y - a.pts[3].y), 2));
    cv::Point2f center_a = getCenter(a);
    cv::Point2f center_l_a = getCenter(l_a);
    double dist = sqrt(pow((center_a.x - center_l_a.x), 2) +
                       pow((center_a.y - center_l_a.y), 2));
    if (dist < thresh)
        return true;
    else
        return false;
}

class DetectorOpencv {
   private:
    ros::Subscriber sensorSub;
    ros::Publisher detectionPub;
    ros::Publisher imgPub;

    cv::Mat img;
    Eigen::Quaternionf q;
    double timestamp;

    std::shared_ptr<CheckBox> detection_checkbox;
    std::shared_ptr<RobotStatus> recv_data;
    std::shared_ptr<RobotStatus> mode_lock;

    int fps, fps_count;
    std::chrono::_V2::system_clock::time_point t1;

   public:
    void onDetect(SmartModel &module) {
        if (this->recv_data->program_mode == ProgramMode::ENERGY_HIT ||
            this->recv_data->program_mode == ProgramMode::ENERGY_DISTURB) {
            this->mode_lock->program_mode = this->recv_data->program_mode;
            std::this_thread::sleep_for(200ms);
            // continue;
        }

        cv::Mat im2show;

        // 先将四点预测结果存储在tmp变量中
        std::vector<bbox_t> result = module(img);
        std::vector<aimer::DetectedArmor> armors;

        // 进行熄灭装甲板处理
        static int cnt = -1;
        static std::vector<bbox_t> last_outputs;
        static std::vector<aimer::DetectedArmor> last_armors;
        static std::vector<aimer::DetectedArmor> tmp_armors;

        if (cnt == -1) {
            last_outputs = result;
            cnt = 1;
        }

        // 存储上一帧的结果
        last_outputs = result;
        // 对结果信息提取到装甲板
        for (const auto& output : result) {
            //            if (output.tag_id == 2) continue; //
            //            不瞄准工程
            cv::Point2f pts[4] = {
                output.pts[0],
                output.pts[1],
                output.pts[2],
                output.pts[3],
            };
            aimer::DetectedArmor a_tmp;
            a_tmp.pts[0] = pts[0];
            a_tmp.pts[1] = pts[1];
            a_tmp.pts[2] = pts[2];
            a_tmp.pts[3] = pts[3];
            a_tmp.color = output.color_id;
            a_tmp.number = output.tag_id == 8 ? 6 : output.tag_id;
            // 归并前哨站和水晶小板
            a_tmp.conf = output.confidence;
            a_tmp.conf_class = output.confidence_cls;
            armors.emplace_back(a_tmp);
        }
        // 根据装甲板大小进行sort
        sort(armors.begin(), armors.end(), cmp_armor_center);

        // TODO: 从多个装甲板中选取目标

        // 存储上一次识别的装甲板结果
        last_armors = armors;

        // 检测结果绘图&显示
        if (this->detection_checkbox->checked) {
            this->fps_count++;
            auto t2 = system_clock::now();
            if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                this->fps = this->fps_count;
                this->fps_count = 0;
                this->t1 = t2;
            }
            // im2show = image.clone();
            im2show = cv::Mat::zeros(cv::Size(1280, 768), CV_8UC1);
            draw_boxes(im2show, result);
            cv::putText(im2show, fmt::format("fps={}", fps), {10, 25},
                        cv::FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255});
            publishImage(im2show);
        }

        // 将检测结果放入FIFO
        publishDetectionResult({img, q, timestamp, armors});
    }

    // void detector_run(const std::string& module_path) {
    void detector_run(const std::string& module_path, ros::NodeHandle& node) {
        // 路径加载四点模型
        SmartModel module(attributes_armor, module_path);
        // publisher and subscriber
        this->sensorSub = node.subscribe<aimer_msgs::SensorsData>(
            "sensors_data", 10, boost::bind(&DetectorOpencv::sensorSubCallback,this,_1,module));
        this->detectionPub =
            node.advertise<aimer_msgs::DetectionResult>("detection_result", 10);
        this->imgPub =
            node.advertise<sensor_msgs::Image>("image-detection", 10);
        //
        this->detection_checkbox =
            umt::ObjManager<CheckBox>::find_or_create("show detections");
        this->recv_data =
            umt::ObjManager<RobotStatus>::find_or_create("robot_status");
        this->mode_lock =
            umt::ObjManager<RobotStatus>::find_or_create("mode_lock");

        // TODO:加载Yolo网络模型

        this->fps = 0, this->fps_count = 0;
        this->t1 = system_clock::now();
        ::base::print(::base::PrintMode::Info, "auto_aim.detector",
                      "即将运行主循环 {}, {}.\n", 2, 123);
        // while (true) {
        //     try {
        //     } catch (umt::MessageError& e) {
        //         fmt::print(fmt::fg(fmt::color::orange),
        //                    "[WARNING] 'sensors_data' {}\n", e.what());
        //         std::this_thread::sleep_for(500ms);
        //     }
        // }
    }

    void sensorSubCallback(const aimer_msgs::SensorsData::ConstPtr& data, SmartModel &module) {
        // sensor_msgs/Image => cv::Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            data->img, sensor_msgs::image_encodings::TYPE_8UC3);
        this->img = cv_ptr->image;
        // time => double
        this->timestamp = data->stamp.toSec();
        // geometry_msgs/Quaternion => Eigen Quaternion
        Eigen::Quaterniond _q;
        tf::quaternionMsgToEigen(data->q, _q);
        Eigen::Vector4d vec=_q.coeffs();
        this->q={(float)vec(0),(float)vec(1),(float)vec(2),(float)vec(3)};
        onDetect(module);
    }

    void publishDetectionResult(aimer::DetectionResult result) {
        aimer_msgs::DetectionResult _result;
        // cv::Mat => sensor_msgs/Image
        sensor_msgs::ImagePtr imagePtr =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", result.img)
                .toImageMsg();
        _result.img = *imagePtr;
        // double => time
        _result.stamp = ros::Time().fromSec(result.timestamp);
        // Eigen Quaterniond => geomotry_msgs/Quaternion
        Eigen::Vector4f vec=result.q.coeffs();
        Eigen::Quaterniond _q={(double)vec(0),(double)vec(1),(double)vec(2),(double)vec(3)};
        tf::quaternionEigenToMsg(_q, _result.q);

        this->detectionPub.publish(_result);
    }

    void publishImage(cv::Mat img) {
        sensor_msgs::ImagePtr imagePtr =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        this->imgPub.publish(imagePtr);
    }
};

void background_detector_run(const std::string& module_path) {
    std::cerr
        << "=========================background_detector_run==============="
           "==============="
        << std::endl;
    std::thread([=]() { /*detector_run(module_path);*/ }).detach();
}

PYBIND11_EMBEDDED_MODULE(aimer_auto_aim_detector, m) {
    namespace py = pybind11;
    m.def("background_detector_run", background_detector_run,
          py::arg("module_path"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "detector");
    ros::NodeHandle node;
    DetectorOpencv detector;

    detector.detector_run("", node);

    ros::spin();
    return 0;
}
