#ifndef OV_CORE_CAM_EUCM_H
#define OV_CORE_CAM_EUCM_H

#include "CamBase.h"
#include <cmath>
#include <cassert>

namespace ov_core {

class CamEUCM : public CamBase {

public:
  CamEUCM(int width, int height) : CamBase(width, height) {
    fx = fy = cx = cy = beta = alpha = 0.0;
    inv_fx = inv_fy = 0.0;
  }

  ~CamEUCM() {}

  virtual void set_value(const Eigen::MatrixXd &calib) override {

    // Expect a 10-vector: fx, fy, cx, cy, _, _, _, _, alpha, beta
    assert(calib.rows() == 10);
    camera_values = calib;

    fx = camera_values(0);
    fy = camera_values(1);
    cx = camera_values(2);
    cy = camera_values(3);

    // user-specified ordering: beta at index 8, alpha at index 9
    alpha = camera_values(8);
    beta = camera_values(9);

    if (inv_fx == 0.0 && inv_fy == 0.0) {
      std::cout << "\033[32m"
                << "CamEUCM intrinsics:"
                << " fx=" << fx << " fy=" << fy
                << " cx=" << cx << " cy=" << cy
                << " alpha=" << alpha << " beta=" << beta
                << "\033[0m" << std::endl;
    }

    inv_fx = (fx != 0.0) ? 1.0 / fx : 0.0;
    inv_fy = (fy != 0.0) ? 1.0 / fy : 0.0;

    // Build OpenCV K matrix
    camera_k_OPENCV = cv::Matx33d(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

    // EUCM does not use OpenCV-style distortion here
    camera_d_OPENCV = cv::Vec4d(0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Undistort raw pixel (u,v) -> normalized coordinates (x_n, y_n)
   * Uses closed-form unprojection for EUCM.
   *
   * Based on Basalt / Extended Unified Camera Model:
   * m_x = (u - cx) / fx
   * m_y = (v - cy) / fy
   * r^2 = m_x^2 + m_y^2
   * m_z = (1 - beta * alpha^2 * r^2) / ( alpha * sqrt(1 - (2*alpha - 1) * beta * r^2) + (1 - alpha) )
   * result: return [m_x/m_z, m_y/m_z]
   */
  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) override {
    const double u = uv_dist(0);
    const double v = uv_dist(1);

    const double mx = (u - cx) * inv_fx;
    const double my = (v - cy) * inv_fy;

    const double r2 = mx * mx + my * my;

    // Domain limit when alpha > 0.5: r2 must be <= 1/(beta*(2*alpha - 1))
    if (alpha > 0.5) {
      const double denom_check = beta * (2.0 * alpha - 1.0);
      if (denom_check <= 0.0) return Eigen::Vector2f(NAN, NAN); // invalid beta/alpha combo
      const double r2_max = 1.0 / denom_check;
      if (r2 > r2_max) {
        return Eigen::Vector2f(NAN, NAN);
      }
    }

    const double tmp = 1.0 - (2.0 * alpha - 1.0) * beta * r2;
    if (tmp < 0.0) return Eigen::Vector2f(NAN, NAN);

    const double sqrt_tmp = std::sqrt(tmp);
    const double denom = alpha * sqrt_tmp + (1.0 - alpha);
    if (denom < 1e-12) return Eigen::Vector2f(NAN, NAN);

    const double mz = (1.0 - beta * alpha * alpha * r2) / denom;

    if (std::abs(mz) < 1e-12) return Eigen::Vector2f(NAN, NAN);

    Eigen::Vector2f zn;
    zn(0) = static_cast<float>(mx / mz);
    zn(1) = static_cast<float>(my / mz);
    return zn;
  }

  /**
   * Distort normalized coords (x_n, y_n) -> raw pixel (u,v)
   * EUCM projection:
   * p = [x, y, 1]
   * d = sqrt(beta * (x^2 + y^2) + z^2)
   * norm = alpha * d + (1 - alpha) * z
   * mx = x / norm
   * my = y / norm
   * u = fx * mx + cx
   * v = fy * my + cy
   */
  Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) override {
    const double x = uv_norm(0);
    const double y = uv_norm(1);
    const double z = 1.0;

    const double r2 = x * x + y * y;
    const double d = std::sqrt(beta * r2 + z * z);

    const double norm = alpha * d + (1.0 - alpha) * z;
    if (std::abs(norm) < 1e-12) return Eigen::Vector2f(NAN, NAN);

    const double mx = x / norm;
    const double my = y / norm;

    Eigen::Vector2f uv;
    uv(0) = static_cast<float>(fx * mx + cx);
    uv(1) = static_cast<float>(fy * my + cy);
    return uv;
  }

  /**
   * Compute analytic Jacobians:
   *  - H_dz_dzn : 2x2 Jacobian of pixel z w.r.t normalized coords [x_n, y_n]
   *  - H_dz_dzeta: 2x10 Jacobian of pixel z w.r.t intrinsics (fx,fy,cx,cy,_,_,_,_,beta,alpha)
   *
   * Implementation follows the Basalt form for EUCM.
   */
  void compute_distort_jacobian(const Eigen::Vector2d &uv_norm,
                                Eigen::MatrixXd &H_dz_dzn,
                                Eigen::MatrixXd &H_dz_dzeta) override {
    // local copies
    const double x = uv_norm(0);
    const double y = uv_norm(1);
    const double z = 1.0;

    const double xx = x * x;
    const double yy = y * y;
    const double r2 = xx + yy;

    const double d = std::sqrt(beta * r2 + z * z);
    const double inv_d = (d > 0.0) ? 1.0 / d : 0.0;

    const double norm = alpha * d + (1.0 - alpha) * z;
    const double norm2 = norm * norm;

    // intermediate for derivatives
    // ∂norm/∂x = alpha * beta * x / d = a * x
    const double a = (d > 0.0) ? (alpha * beta * inv_d) : 0.0;
    // ∂norm/∂z = alpha * z / d + (1 - alpha)
    const double dnorm_dz = (d > 0.0) ? (alpha * z * inv_d + (1.0 - alpha)) : (1.0 - alpha);

    // Build d_proj_d_p3d (2x3)
    Eigen::Matrix<double, 2, 3> d_proj_d_p3d;
    d_proj_d_p3d.setZero();

    // Using chain rule:
    // ∂(fx * x / norm) / ∂x = fx * (1/norm - x * (∂norm/∂x) / norm^2) ...
    d_proj_d_p3d(0, 0) = fx * (1.0 / norm - (a * xx) / norm2);
    d_proj_d_p3d(0, 1) = -fx * (a * x * y) / norm2;
    d_proj_d_p3d(0, 2) = -fx * x * dnorm_dz / norm2;

    d_proj_d_p3d(1, 0) = -fy * (a * x * y) / norm2;
    d_proj_d_p3d(1, 1) = fy * (1.0 / norm - (a * yy) / norm2);
    d_proj_d_p3d(1, 2) = -fy * y * dnorm_dz / norm2;

    // d_proj_d_param (2x6) for [fx, fy, cx, cy, alpha, beta]
    Eigen::Matrix<double, 2, 6> d_proj_d_param;
    d_proj_d_param.setZero();

    // fx, fy, cx, cy direct partials
    const double mx = x / norm;
    const double my = y / norm;
    d_proj_d_param(0, 0) = mx;  // d u / d fx
    d_proj_d_param(1, 1) = my;  // d v / d fy
    d_proj_d_param(0, 2) = 1.0; // d u / d cx
    d_proj_d_param(1, 3) = 1.0; // d v / d cy

    // Partial derivative wrt alpha:
    // ∂norm/∂alpha = d - z
    const double d_minus_z = d - z;
    d_proj_d_param(0, 4) = -fx * x * (d_minus_z) / norm2;
    d_proj_d_param(1, 4) = -fy * y * (d_minus_z) / norm2;

    // Partial derivative wrt beta:
    // ∂d/∂beta = r2 / (2 d)  => ∂norm/∂beta = alpha * r2 / (2 d)
    const double dnorm_d_beta = (d > 0.0) ? (alpha * r2 * 0.5 * inv_d) : 0.0;
    d_proj_d_param(0, 5) = -fx * x * dnorm_d_beta / norm2;
    d_proj_d_param(1, 5) = -fy * y * dnorm_d_beta / norm2;

    // H_dz_dzn = d_proj_d_p3d * [I2; 0] => take first two columns
    H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
    H_dz_dzn(0, 0) = d_proj_d_p3d(0, 0);
    H_dz_dzn(0, 1) = d_proj_d_p3d(0, 1);
    H_dz_dzn(1, 0) = d_proj_d_p3d(1, 0);
    H_dz_dzn(1, 1) = d_proj_d_p3d(1, 1);

    // camera_values = [ fx, fy, cx, cy, _, _, _, _, beta, alpha ]
    // d_proj_d_param is [fx, fy, cx, cy, alpha, beta] -> place beta,alpha into cols 8,9
    H_dz_dzeta = Eigen::MatrixXd::Zero(2, 10);
    H_dz_dzeta.block<2, 4>(0, 0) = d_proj_d_param.block<2, 4>(0, 0);

    // put beta into column 8 (camera_values index 8)
    H_dz_dzeta.col(8) = d_proj_d_param.col(5); // beta
    // put alpha into column 9 (camera_values index 9)
    H_dz_dzeta.col(9) = d_proj_d_param.col(4); // alpha
  }

private:
  // explicit intrinsics
  double fx;
  double fy;
  double cx;
  double cy;
  double alpha;
  double beta;

  // cached helpers
  double inv_fx;
  double inv_fy;
};

} // namespace ov_core

#endif /* OV_CORE_CAM_EUCM_H */
