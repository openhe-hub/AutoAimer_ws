#include <array>
#include <opencv2/opencv.hpp>

/*
result 为cv::Point2f pts[4]数组的指针
input_image为待处理图像
void find_green_light_traditional(cv::Point2f *result,
                                 const cv::Mat imput_image) {}
*/
int My_Abs(int x);
cv::Mat Reverse(cv::Mat &src); //颜色翻转
cv::Mat HSV_green_rev_and_white(const cv::Mat &src);
cv::Mat HSV_green(const cv::Mat &src);
cv::Point2f find_green_centre(cv::Mat &img_b, cv::Mat &hsv_green);
bool Judge_by_shape(float width, float height, float thres);
bool Judge_by_pos(int x, int xx, int y, int yy, int rows, int cols, float t);
bool find_draw_hsv(cv::Mat &img_b, cv::Mat &hsv, cv::Mat &img_find,
                   cv::Point2f &green_centre, cv::Point2f *result);

bool find_green_light_traditional(cv::Point2f *result,
                                  const cv::Mat &input_image) {
  cv::Mat img_hsv, hsv_green, img_rgb, img_find, img_b;

  img_b = input_image;

  if (img_b.empty()) {
    return false;
  }
  img_hsv = HSV_green_rev_and_white(img_b);
  hsv_green = HSV_green(img_b);

  // cv::imshow("src",img_b);
  // cv::imshow("hsv",img_hsv);
  // cv::waitKey(0);
  cv::Point2f green_centre = find_green_centre(img_b, hsv_green);
  if (!find_draw_hsv(img_b, img_hsv, img_find, green_centre, result)) {
    return false;
  } else {
    return true;
  }
}
int My_Abs(int x) { return x < 0 ? -x : x; }
bool Judge_by_shape(float width, float height, float thres) {
  if (1. * width / height >= thres && 1. * height / width >= thres) {
    return true;
  } else {
    return false;
  }
}
bool Judge_by_pos(int x, int xx, int y, int yy, int rows, int cols, float t) {
  std::swap(cols, rows);
  if (1. * x / cols > t && 1. * y / rows > t && 1. * xx / cols < 1 - t &&
      1. * yy / rows < 1 - t) {
    return true;
  } else {
    return false;
  }
}
cv::Mat Reverse(cv::Mat &src) {
  cv::Mat ones = cv::Mat::ones(cv::Size(src.cols, src.rows), CV_8UC1);
  cv::Mat res = 255 * ones - src;
  return res;
}
cv::Mat HSV_green_rev_and_white(const cv::Mat &src) {
  cv::Mat hsv;
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

  cv::Mat hsv_white, hsv_green;
  cv::inRange(hsv, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255), hsv_white);
  cv::inRange(hsv, cv::Scalar(60, 43, 46), cv::Scalar(90, 255, 255), hsv_green);

  cv::Mat hsv_green_rev;
  hsv_green_rev = Reverse(hsv_green);
  // cv::imshow("111",hsv_green);

  cv::Mat hsv_res = cv::Mat::ones(cv::Size(src.cols, src.rows), CV_8UC1);

  // hsv_res = 255 * (hsv_res - (hsv_res - hsv_white/255).mul(hsv_res -
  // hsv_green_inv/255));
  hsv_res = hsv_green_rev & hsv_white;
  cv::Mat Morph_Result1, Morph_Result2, Morph_Result3;
  cv::Mat kernel1 =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
  cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT,
  // cv::Size(40,40));

  cv::morphologyEx(hsv_res, Morph_Result1, cv::MORPH_CLOSE, kernel1);
  cv::morphologyEx(Morph_Result1, Morph_Result2, cv::MORPH_OPEN, kernel2);

  // cv::imshow("white", hsv_white);
  // cv::imshow("green_inv", hsv_res);
  // cv::imshow("res", hsv_res);
  return Morph_Result2;
}
cv::Mat HSV_green(const cv::Mat &src) {
  cv::Mat hsv;
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

  cv::Mat hsv_green;
  cv::inRange(hsv, cv::Scalar(60, 43, 46), cv::Scalar(80, 255, 255), hsv_green);

  cv::Mat Morph_Result1, Morph_Result2, Morph_Result3;
  cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(4, 4));
  cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(6, 6));
  // cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT,
  // cv::Size(40,40));

  cv::morphologyEx(hsv_green, Morph_Result1, cv::MORPH_CLOSE, kernel1);
  // cv::morphologyEx(Morph_Result1, Morph_Result2, cv::MORPH_CLOSE, kernel2);
  return Morph_Result1;
}

bool find_draw_hsv(cv::Mat &img_b, cv::Mat &hsv, cv::Mat &img_find,
                   cv::Point2f &green_centre, cv::Point2f *result) {

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Point2f rect[4];
  cv::RotatedRect minE;
  img_find = img_b.clone();
  // find outline
  findContours(hsv, contours, hierarchy, cv::RETR_EXTERNAL,
               cv::CHAIN_APPROX_NONE);

  std::vector<cv::RotatedRect> box(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());

  int contour_cnt = 0;
  int matched_light = -1;
  int Min_dist = 0x3f3f3f3f;
  if (contours.empty()) {
    // std::cout << "find no light!" << std::endl;
    return false;
  } else {
    // std::cout << "succeed finding light!" << std::endl;
    // std::cout << "green_centre" << green_centre.x << ' ' << green_centre.y
    //         << std::endl;
    for (int i = 0; i < (int)contours.size(); i++) {
      box[i] = cv::minAreaRect(cv::Mat(contours[i])); //计算每个轮廓最小外接矩形
      boundRect[i] = cv::boundingRect(cv::Mat(contours[i]));
      if (box[i].size.width * box[i].size.height < 100)
        continue;
      if (box[i].size.width * box[i].size.height > 1500)
        continue;
      contour_cnt++;

      int x = boundRect[i].x, y = boundRect[i].y;
      int xx = boundRect[i].x + boundRect[i].width,
          yy = boundRect[i].y + boundRect[i].height;

      if (Judge_by_shape(box[i].size.width, box[i].size.height,
                         .7)) { // bounding_rectangle ~= square
        if (Judge_by_pos(x, xx, y, yy, img_b.cols, img_b.rows,
                         .0)) { // contour is in the centre of the image
          int now_dist = My_Abs((int)box[i].center.x - green_centre.x) +
                         My_Abs((int)box[i].center.y - green_centre.y);

          if (now_dist < Min_dist) {

            Min_dist = now_dist;
            matched_light = i;
          }
        }
      }
      // cv::imwrite("match_light.jpg",img_find);
    }
    if (matched_light != -1) {
      circle(
          img_find,
          cv::Point(box[matched_light].center.x, box[matched_light].center.y),
          5, cv::Scalar(0, 0, 255), -1, 8); //绘制最小外接矩形的中心点
      box[matched_light].points(rect); //把最小外接矩形四个端点复制给rect数组
      rectangle(
          img_find,
          cv::Point(boundRect[matched_light].x, boundRect[matched_light].y),
          cv::Point(boundRect[matched_light].x + boundRect[matched_light].width,
                    boundRect[matched_light].y +
                        boundRect[matched_light].height),
          cv::Scalar(0, 255, 0), 2, 8);
      // std::cout << matched_light << std::endl;

      int x = boundRect[matched_light].x, y = boundRect[matched_light].y;
      int xx = boundRect[matched_light].x + boundRect[matched_light].width,
          yy = boundRect[matched_light].y + boundRect[matched_light].height;

      result[0].x = x;
      result[0].y = y;
      result[1].x = xx;
      result[1].y = y;
      result[2].x = xx;
      result[2].y = yy;
      result[3].x = x;
      result[3].y = yy;
      // cv::imshow("img_find", img_find);
      // cv::waitKey(20);
      return true;
    } else {
      // std::cout << "find no light!" << std::endl;
      return false;
    }
  }
}

cv::Point2f find_green_centre(cv::Mat &img_b, cv::Mat &hsv_green) {
  cv::Mat img_tmp = img_b.clone();
  cv::Point2f green_centre;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Point2f rect[4];
  cv::RotatedRect minE;
  // find outline
  findContours(hsv_green, contours, hierarchy, cv::RETR_EXTERNAL,
               cv::CHAIN_APPROX_NONE);

  std::vector<cv::RotatedRect> box(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());

  int contour_cnt = 0;
  int matched_light = -1;
  double Max_Size = 50;
  if (contours.empty()) {
    green_centre.x = hsv_green.cols / 2;
    green_centre.y = hsv_green.rows / 2;
    return green_centre;
  } else {
    // std::cout << "succeed finding light!" << std::endl;
    for (int i = 0; i < (int)contours.size(); i++) {
      box[i] = cv::minAreaRect(cv::Mat(contours[i])); //计算每个轮廓最小外接矩形
      boundRect[i] = cv::boundingRect(cv::Mat(contours[i]));
      if (box[i].size.width * box[i].size.height < 100) continue;
      if (box[i].size.width * box[i].size.height > 2500) continue;
      contour_cnt++;

      int x = boundRect[i].x, y = boundRect[i].y;
      int xx = boundRect[i].x + boundRect[i].width,
          yy = boundRect[i].y + boundRect[i].height;

      if (Judge_by_shape(box[i].size.width, box[i].size.height,
                         .5)) { // bounding_rectangle ~= square
        if (Judge_by_pos(x, xx, y, yy, img_b.cols, img_b.rows,
                         .0)) { // contour is in the centre of the image
          double now_size = box[i].size.height * box[i].size.width;

          if (now_size > Max_Size) {

            Max_Size = now_size;
            matched_light = i;
            green_centre = box[i].center;
          }
        }
      }
    }
    if (matched_light != -1) {

      circle(
          img_tmp,
          cv::Point(box[matched_light].center.x, box[matched_light].center.y),
          5, cv::Scalar(255, 0, 0), -1, 8); //绘制最小外接矩形的中心点
      box[matched_light].points(rect); //把最小外接矩形四个端点复制给rect数组
      rectangle(
          img_tmp,
          cv::Point(boundRect[matched_light].x, boundRect[matched_light].y),
          cv::Point(boundRect[matched_light].x + boundRect[matched_light].width,
                    boundRect[matched_light].y +
                        boundRect[matched_light].height),
          cv::Scalar(255, 0, 0), 2, 8);
      // std::cout << Max_Size << std::endl;
      // cv::imshow("green_centre", img_tmp);
      return green_centre;
    } else {
      green_centre.x = hsv_green.cols / 2;
      green_centre.y = hsv_green.rows / 2;
      return green_centre;
    }
  }
}