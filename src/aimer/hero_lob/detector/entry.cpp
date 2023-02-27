#include <chrono>
#include <thread>

#include <fmt/color.h>
#include <fmt/format.h>
#include <pybind11/numpy.h>
#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "UltraMultiThread/include/umt/umt.hpp"
#include "aimer/hero_lob/detector/traditional_vision.hpp"
#include "aimer/hero_lob/hero_defs.hpp"
#include "common/common.hpp"
#include "core_io/robot.hpp"
#include "core_io/sensors.hpp"
using namespace std::chrono;
cv::Scalar color_map[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
void draw_boxes(cv::Mat& src, cv::Point2f* results) {
  const auto& color = color_map[0];
  cv::line(src, results[0], results[1], color, 1);
  cv::line(src, results[1], results[2], color, 1);
  cv::line(src, results[2], results[3], color, 1);
  cv::line(src, results[3], results[0], color, 1);
  // cv::putText(src, std::to_string(box.tag_id), box.pts[0], 0, 1, color);
  // cv::putText(src, std::to_string(box.confidence).substr(0, 4), box.pts[2],
  // 0,
  //             1, color);
  // cv::putText(src, "c" + std::to_string(box.confidence_cls).substr(0, 4),
  //             box.pts[3], 0, 1, color);
}
void green_light_detector_run() {
  // 创建相机陀螺仪数据接收者
  umt::Subscriber<SensorsData> subscriber("sensors_data");
  // 创建识别结果发布者
  umt::Publisher<hero::GreenLightDetectionResult> publisher(
      "green_light_detections");
  auto detection_checkbox =
      umt::ObjManager<CheckBox>::find_or_create("hero_lob show detections");
  umt::Publisher<cv::Mat> detections_client("hero_lob-image-detections");

  int fps = 0, fps_count = 0;
  auto t1 = system_clock::now();
  while (true) {
    try {
      const auto& [image, q_, timestamp] = subscriber.pop();

      Eigen::Quaternionf q(q_[0], q_[1], q_[2], q_[3]);
      hero::GreenLightDetectionResult tmp;
      cv::Point2f pts[4];
      // if(!find_green_light_traditional(pts, image))
      // continue;
      bool found = find_green_light_traditional(pts, image);
      // find_green_light_neural(pts, image);

      cv::Mat im2show;
      if (detection_checkbox->checked) {
        fps_count++;
        auto t2 = system_clock::now();
        if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
          fps = fps_count;
          fps_count = 0;
          t1 = t2;
        }
        im2show = image.clone();
        draw_boxes(im2show, pts);
        cv::putText(im2show, fmt::format("fps={}", fps), {10, 25},
                    cv::FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255});
        detections_client.push(im2show);
      }
      tmp.pts[0] = pts[0];
      tmp.pts[1] = pts[1];
      tmp.pts[2] = pts[2];
      tmp.pts[3] = pts[3];
      tmp.img = image;
      tmp.q = q;
      tmp.timestamp = timestamp;
      tmp.found=found;
      publisher.push(tmp);
    } catch (umt::MessageError& e) {
      fmt::print(fmt::fg(fmt::color::orange), "[WARNING] 'sensors_data' {}\n",
                 e.what());
      std::this_thread::sleep_for(500ms);
    }
  }
}

void background_green_light_detector_run() {
  std::cerr << "=========================background_green_light_detector_run==="
               "============"
               "==============="
            << std::endl;
  std::thread([=]() { green_light_detector_run(); }).detach();
}

PYBIND11_EMBEDDED_MODULE(Heroaim, m) {
  //   namespace py = pybind11;
  m.def("background_green_light_detector_run",
        background_green_light_detector_run);
  //   m.def("background_detector_run", background_detector_run,
  //         py::arg("module_path"));
}
