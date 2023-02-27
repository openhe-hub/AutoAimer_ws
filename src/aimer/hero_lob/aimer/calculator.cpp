#include <math.h>
#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <utility>
#include <vector>
#include "aimer/param/parameter.hpp"

using namespace std;
namespace hero {
// const double m = 0.041;
// const double k = 1;
vector<double> x;
vector<double> y;
vector<double> theta;
vector<double> v;
double alpha = 0;
double h = 0.01;  // 步长在这里
const double g = 9.7940;
double cnt = 0;
double x_target = -1, y_target = -1, M = -1;
//初始化
void init(double h_new,
          double x_new,
          double y_new,
          double theta_new,
          double v_new) {
  /*
  description:用于初始化初始值
  */

  h = h_new;
  x.clear();
  y.clear();
  v.clear();
  theta.clear();
  x.push_back(x_new);
  y.push_back(y_new);
  theta.push_back(theta_new);
  v.push_back(v_new);
  cnt = 0;
}

// 定义x导的微分方程
double x_deriv(double v_f, double theta_f) {
  double x1 = v_f * cos(theta_f);
  return x1;
}

//定义V导的微分方程
double v_deriv(double v_f, double theta_f, double y_f) {
  Eigen::RowVector3d a((v_f / 340.) * (v_f / 340.), v_f / 340., 1);
  Eigen::Matrix3d b;
  double k = aimer::param::find_double_param("HERO_BULLET_RESISTANCE_K");
  b << 0.0002, 0.0038, 0.1582, -0.0022, -0.0132, -0.8520, 0.0115, -0.0044,
      k;
  Eigen::Vector3d c(alpha * alpha, alpha, 1);
  double Cx = a * b * c;
  double ro = 1.225 * exp(-0.00015 * y_f);
  double s = aimer::param::find_double_param("HERO_BULLET_SIZE");
  double D = 0.5*ro*s*Cx * v_f * v_f;
  double m = aimer::param::find_double_param("BULLET_RESISTANCE_M");
  double v1 = -D / m - g * sin(theta_f);
  return v1;
}
// 定义y导的微分方程
double y_deriv(double v_f, double theta_f) {
  double y1 = v_f * sin(theta_f);
  return y1;
}

//定义θ导的微分方程
double theta_deriv(double v_f, double theta_f, double y_f) {
    Eigen::RowVector3d a((v_f / 340.) * (v_f / 340.), v_f / 340., 1);
    Eigen::Vector3d b(-0.026,0.0651,0.4913);
    double Cy = a* b;
    Cy*= alpha;
   double ro = 1.225 * exp(-0.00015 * y_f);
  double s = aimer::param::find_double_param("HERO_BULLET_SIZE");
   double  L = 0.5*ro*v_f*v_f*s*Cy;
  double m = aimer::param::find_double_param("BULLET_RESISTANCE_M");
  double theta1 = -g * cos(theta_f) / v_f + L / (m * v_f);
  return theta1;
}
struct param {
  double x1, y1, v1, theta1;
};
// 定义全部导数
param param_deriv(double v_f, double theta_f, double y_f) {
  /*
  description:同时对x,y,v,theta进行求导
  */
  double x1 = x_deriv(v_f, theta_f);
  double y1 = y_deriv(v_f, theta_f);
  double v1 = v_deriv(v_f, theta_f, y_f);
  double theta1 = theta_deriv(v_f, theta_f, y_f);
  param ans;
  ans.x1 = x1;
  ans.y1 = y1;
  ans.v1 = v1;
  ans.theta1 = theta1;
  return ans;
}
//定义Runge-Kutta方程
param iter(double step,
           double x0,
           double y0,
           double v0,
           double theta0,
           double kx1,
           double ky1,
           double kv1,
           double ktheta1) {
  /*
  description : 龙格库塔的K的一次更新
  */
  double x1 = x0 + 0.5 * step * kx1;
  double y1 = y0 + 0.5 * step * ky1;
  double v1 = v0 + 0.5 * step * kv1;
  double theta1 = theta0 + 0.5 * step * ktheta1;
  param ans;
  ans.x1 = x1;
  ans.y1 = y1;
  ans.v1 = v1;
  ans.theta1 = theta1;
  return ans;
}
param RungeKutta(double step) {
  /*
  description:龙格库塔的一次更新
  */
  double v0 = v.back();
  double x0 = x.back();
  double y0 = y.back();
  double theta0 = theta.back();
  param tmp;
  tmp = param_deriv(v0, theta0, y0);
  double kx1 = tmp.x1;
  double ky1 = tmp.y1;
  double kv1 = tmp.v1;
  double ktheta1 = tmp.theta1;
  tmp = iter(step, x0, y0, v0, theta0, kx1, ky1, kv1, ktheta1);
  // double x1 = tmp.x1;
  double y1 = tmp.y1;
  double v1 = tmp.v1;
  double theta1 = tmp.theta1;
  tmp = param_deriv(v1, theta1, y1);
  double kx2 = tmp.x1;
  double ky2 = tmp.y1;
  double kv2 = tmp.v1;
  double ktheta2 = tmp.theta1;
  tmp = iter(step, x0, y0, v0, theta0, kx2, ky2, kv2, ktheta2);
  // double x2 = tmp.x1;
  double y2 = tmp.y1;
  double v2 = tmp.v1;
  double theta2 = tmp.theta1;
  tmp = param_deriv(v2, theta2, y2);
  double kx3 = tmp.x1;
  double ky3 = tmp.y1;
  double kv3 = tmp.v1;
  double ktheta3 = tmp.theta1;
  tmp = iter(step, x0, y0, v0, theta0, 2 * kx3, 2 * ky3, 2 * kv3, 2 * ktheta3);
  // double x3 = tmp.x1;
  double y3 = tmp.y1;
  double v3 = tmp.v1;
  double theta3 = tmp.theta1;
  tmp = param_deriv(v3, theta3, y3);
  double kx4 = tmp.x1;
  double ky4 = tmp.y1;
  double kv4 = tmp.v1;
  double ktheta4 = tmp.theta1;
  param ans;
  double v_t = v0 + step / 6 * (kv1 + 2 * kv2 + 2 * kv3 + kv4);
  double theta_t =
      theta0 + step / 6 * (ktheta1 + 2 * ktheta2 + 2 * ktheta3 + ktheta4);
  double x_t = x0 + step / 6 * (kx1 + 2 * kx2 + 2 * kx3 + kx4);
  double y_t = y0 + step / 6 * (ky1 + 2 * ky2 + 2 * ky3 + ky4);
  ans.x1 = x_t;
  ans.y1 = y_t;
  ans.v1 = v_t;
  ans.theta1 = theta_t;
  // cout << "now pos: " << x_t << " " << y_t << endl;
  return ans;
}

//计算弹道
pair<double, int> calculate(double step,
                            double x_new,
                            double y_new,
                            double theta_new,
                            double v_new) {
  /*
   description: 判断落地为y是否大于0,落地后跳出循环，一次循环即一次迭代。
                先同时做一次龙格库塔,然后依次更新。
                用于定步长计算。
   */
  init(step, x_new, y_new, theta_new, v_new);
  double min_dist = 99999999;
  while (y.back() > 0) {
    param tmp;
    tmp = RungeKutta(step);
    x.push_back(tmp.x1);
    y.push_back(tmp.y1);
    v.push_back(tmp.v1);
    theta.push_back(tmp.theta1);
    M = v.back() / 340;
    min_dist =
        min(min_dist, sqrt((x_target - x.back()) * (x_target - x.back()) +
                           (y_target - y.back()) * (y_target - y.back())));
    if (x.back() > x_target && y.back() > y_target) {
      // print("Hit speed = " + str(v[-1]) + "m/s");
      return make_pair(min_dist, 1);
    } else if (x.back() > x_target && y.back() < y_target) {
      // print("Hit speed = " + str(v[-1]) + "m/s");
      return make_pair(min_dist, 0);
    }
    cnt += 1;
  }
  // print('命中用时：'+str(round(cnt*step*10)/10)+'s')
  // print("Hit speed = " + str(v[-1]) + "m/s");
  return make_pair(min_dist, 0);
}
pair<double, int> RungeKutta_fixed(double step,
                                   double x_new,
                                   double y_new,
                                   double theta_new,
                                   double v_new) {
  pair<double, int> ret = calculate(step, x_new, y_new, theta_new, v_new);
  return ret;
}
double dandaojiesuan(double x_tar, double y_tar, double Ma_he) {
  alpha = aimer::param::find_double_param("HERO_BULLET_ALPHA");
  h = 0.01;  // 步长在这里
  cnt = 0;
  x_target = x_tar;
  y_target = y_tar;
  M = Ma_he / 340.;
  cout << "x: " << x_target << "  y: " << y_target << " mahe:" << Ma_he << endl;
  double l = 0., r = 90., bst_deg = 45., dist = 9999999;
  for (int i = 1; i < 50; i++) {
    double now_deg = (l + r) / 2.;
    double now_rad = now_deg / 180 * M_PI;
    // std::cout << now_deg << endl << endl;
    pair<double, int> res =
        RungeKutta_fixed(0.01, 0.1275 * cos(now_rad), 0.1275 * sin(now_rad),
                         now_deg / 360 * 2 * M_PI, Ma_he);
    cout << "x: " << x_target - 0.1275 * cos(now_rad)<< "  y: " << y_target - 0.1275 * sin(now_rad)<< " mahe:" << Ma_he << endl;
    if (res.second == 0)
      l = now_deg;
    else
      r = now_deg;
    double now_dist = res.first;
    // std::cout << now_dist << endl;
    if (now_dist < dist) {
      dist = now_dist;
      bst_deg = now_deg;
    }
    // std::cout << now_deg << " " << dist << endl;
  }
  cout << "bst_ang: " << bst_deg << " "
       << aimer::param::find_double_param("BULLET_RESISTANCE_M") << " "
       << aimer::param::find_double_param("HERO_BULLET_SIZE") << endl;
  return bst_deg;
}
}  // namespace hero