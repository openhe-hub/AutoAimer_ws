#ifndef _Traditional_Find_Light_
#define _Traditional_Find_Light_
#include <opencv2/opencv.hpp>

// result 为cv::Point2f pts[4]数组的指针
// input_image为待处理图像
bool find_green_light_traditional(cv::Point2f *result,
                                  const cv::Mat &imput_image);

#endif