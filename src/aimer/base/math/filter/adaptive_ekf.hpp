//
// Created by zzy on 7/9/21.
//

#ifndef PREDICTOR_ADAPTIVE_EKF_HPP
#define PREDICTOR_ADAPTIVE_EKF_HPP

#include <ceres/jet.h>
#include <Eigen/Dense>

namespace aimer {

/** @class 扩展卡尔曼滤波，并不 Adaptive（自适应）.
 *
 * \brief name associated with N_X, N_Y.
 *
 * N_X = 6, N_Y = 3, for example.
 */
template <int N_X, int N_Y>
class AdaptiveEkf {
  //  private:
  // using This = AdaptiveEkf<N_X, N_Y>;

 public:
  using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
  using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
  using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
  using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
  using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
  using MatrixY1 = Eigen::Matrix<double, N_Y, 1>;

 private:
  MatrixX1 x_e;    // 估计状态变量
  MatrixXX p_mat;  // 状态协方差

  static constexpr double INF{1e9};

 public:
  AdaptiveEkf()
      : x_e{AdaptiveEkf::MatrixX1::Zero()},
        p_mat{AdaptiveEkf::MatrixXX::Identity() * INF} {}

  // 初始化且方差为单位矩阵
  void init_x(const AdaptiveEkf::MatrixX1& x0) {
    this->x_e = x0;
    this->p_mat = AdaptiveEkf::MatrixXX::Identity();
  }

  // 修正 y 用
  // MatrixX1 get_x() const { return this->x_e; }

  struct PredictResult {
    // private:
    // using This = PredictResult;
    AdaptiveEkf::MatrixX1 x_p;
    AdaptiveEkf::MatrixXX f_mat;
  };

  template <class PredictFunc>
  PredictResult predict(PredictFunc&& predict_func) const {
    ceres::Jet<double, N_X> x_e_jet[N_X];
    for (int i{0}; i < N_X; ++i) {
      x_e_jet[i].a = this->x_e[i];
      x_e_jet[i].v[i] = 1.;
      // a 对自己的偏导数为 1.
    }
    ceres::Jet<double, N_X> x_p_jet[N_X];
    predict_func(x_e_jet, x_p_jet);
    AdaptiveEkf::MatrixX1 x_p{AdaptiveEkf::MatrixX1::Zero()};
    for (int i{0}; i < N_X; ++i) {
      x_p[i] = x_p_jet[i].a;
    }
    AdaptiveEkf::MatrixXX f_mat{AdaptiveEkf::MatrixXX::Zero()};
    for (int i{0}; i < N_X; ++i) {
      f_mat.block(i, 0, 1, N_X) = x_p_jet[i].v.transpose();
    }
    return AdaptiveEkf::PredictResult{x_p, f_mat};
  }

  template <class PredictFunc>
  void predict_forward(PredictFunc&& predict_func,
                       const AdaptiveEkf::MatrixXX& q_mat) {
    AdaptiveEkf::PredictResult pre_res{this->predict(predict_func)};
    this->x_e = pre_res.x_p;
    this->p_mat =
        pre_res.f_mat * this->p_mat * pre_res.f_mat.transpose() + q_mat;
  }

  struct MeasureResult {
    AdaptiveEkf::MatrixY1 y_e;
    AdaptiveEkf::MatrixYX h_mat;
  };

  template <class MeasureFunc>
  MeasureResult measure(MeasureFunc&& measure_func) {
    ceres::Jet<double, N_X> x_e_jet[N_X];
    for (int i{0}; i < N_X; ++i) {
      x_e_jet[i].a = this->x_e[i];
      x_e_jet[i].v[i] = 1;
    }
    ceres::Jet<double, N_X> y_e_jet[N_Y];
    measure_func(x_e_jet, y_e_jet);  // 转化成 Y 类型后的预测值，期间自动求导
    AdaptiveEkf::MatrixY1 y_e{AdaptiveEkf::MatrixY1::Zero()};
    for (int i{0}; i < N_Y; ++i) {
      y_e[i] = y_e_jet[i].a;
    }
    AdaptiveEkf::MatrixYX h_mat{AdaptiveEkf::MatrixYX::Zero()};
    for (int i{0}; i < N_Y; ++i) {
      h_mat.block(i, 0, 1, N_X) = y_e_jet[i].v.transpose();
    }
    return AdaptiveEkf::MeasureResult{y_e, h_mat};
  }

  template <class MeasureFunc>
  void update_forward(MeasureFunc&& measure_func,
                      const AdaptiveEkf::MatrixY1& y_mat,
                      const AdaptiveEkf::MatrixYY& r_mat) {
    AdaptiveEkf::MeasureResult mea_res{this->measure(measure_func)};
    // K 中包含 Y 到 X 的一阶转移矩阵
    AdaptiveEkf::MatrixXY k_mat{
        this->p_mat * mea_res.h_mat.transpose() *
        (mea_res.h_mat * this->p_mat * mea_res.h_mat.transpose() + r_mat)
            .inverse()};
    this->x_e = this->x_e + k_mat * (y_mat - mea_res.y_e);
    this->p_mat = (AdaptiveEkf::MatrixXX::Identity() - k_mat * mea_res.h_mat) *
                  this->p_mat;
  }

  template <class MeasureFunc, class PredictFunc>
  void update(MeasureFunc&& measure_func,
              PredictFunc&& predict_func,
              const AdaptiveEkf::MatrixY1& y_mat,
              const AdaptiveEkf::MatrixXX& q_mat,
              const AdaptiveEkf::MatrixYY& r_mat) {
    this->predict_forward(predict_func, q_mat);
    this->update_forward(measure_func, y_mat, r_mat);
  }
};
}  // namespace aimer
#endif  // PREDICTOR_ADAPTIVE_EKF_HPP
