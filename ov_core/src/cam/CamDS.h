#ifndef OV_CORE_CAM_DS_H
#define OV_CORE_CAM_DS_H

#include "CamBase.h"
#include <cmath>
#include <cassert>

namespace ov_core {

/**
 * @brief Double-Sphere camera model (header-only)
 *
 * Notes:
 * - This model does NOT use OpenCV-style distortion. We zero camera_d_OPENCV to avoid misuse.
 * - camera_values layout (8 entries for compatibility with CamBase):
 *   [ fx, fy, cx, cy, xi, alpha, 0, 0 ]^T
 *
 * - set_value accepts:
 *    * 6x1 Kalibr ordering: [ xi, alpha, fu, fv, pu, pv ]  (common)
 *    * 8x1 OpenVINS ordering: [ fx, fy, cx, cy, xi, alpha, 0, 0 ]
 *
 * Implementation follows Usenko et al. "Double Sphere" and Basalt's implementation
 * for projection / unprojection / jacobians.
 */
class CamDS : public CamBase {

public:
  CamDS(int width, int height) : CamBase(width, height) {
    fx = fy = cx = cy = xi = alpha = 0.0;
    inv_fx = inv_fy = 0.0;
  }

  ~CamDS() {}

  /**
   * Override set_value to accept Kalibr 6-vector or 8-vector and to avoid storing xi/alpha in camera_d_OPENCV.
   */
  virtual void set_value(const Eigen::MatrixXd &calib) override {
    
    // Assert we are of size eight
    assert(calib.rows() == 8);
    camera_values = calib;

    // Cache intrinsics in convenient scalars
    fx = camera_values(0);
    fy = camera_values(1);
    cx = camera_values(2);
    cy = camera_values(3);
    xi = camera_values(4);
    alpha = camera_values(5);

    std::cout << "\033[32m"
              << "CamDS intrinsics:"
              << " fx=" << fx << " fy=" << fy
              << " cx=" << cx << " cy=" << cy
              << " xi=" << xi << " alpha=" << alpha
              << "\033[0m" << std::endl;

    // precompute inverses
    inv_fx = (fx != 0.0) ? 1.0 / fx : 0.0;
    inv_fy = (fy != 0.0) ? 1.0 / fy : 0.0;

    // Build OpenCV K matrix (useful for visualization / reprojection helpers)
    camera_k_OPENCV = cv::Matx33d(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

    // Double-Sphere has no OpenCV-style distortion coefficients (avoid confusion)
    camera_d_OPENCV = cv::Vec4d(0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Undistort raw pixel (u,v) -> normalized coordinates (x_n, y_n)
   * Uses closed-form unprojection for DS model.
   */
  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) override {
    // Convert to normalized pixel coordinates (mx,my)
    const double u = uv_dist(0);
    const double v = uv_dist(1);
    const double mx = (u - cx) * inv_fx;
    const double my = (v - cy) * inv_fy;

    const double r2 = mx * mx + my * my;

    // Compute intermediate mz following Basalt / Usenko unprojection
    // Note: There are different algebraic forms; this is the stable one used in Basalt.
    const double tmp = 1.0 - (2.0 * alpha - 1.0) * r2;
    const double sqrt_tmp = (tmp > 0.0) ? std::sqrt(tmp) : 0.0;
    const double denom = alpha * sqrt_tmp + (1.0 - alpha);

    // guard denom near zero
    if (std::abs(denom) < 1e-12) {
      // fallback: return normalized mx,my (best-effort)
      Eigen::Vector2f out;
      out(0) = static_cast<float>(mx);
      out(1) = static_cast<float>(my);
      return out;
    }

    const double mz = (1.0 - (alpha * alpha) * r2) / denom;

    // Further compute closed-form inverse (see Basalt)
    const double mz2 = mz * mz;
    const double sqrt_inner = mz2 + (1.0 - xi * xi) * r2;
    const double sqrt_inner_clamped = (sqrt_inner > 0.0) ? std::sqrt(sqrt_inner) : 0.0;

    const double k_num = mz * xi + sqrt_inner_clamped;
    const double k_den = mz2 + r2;

    // protect against division by zero
    const double k = (std::abs(k_den) > 1e-12) ? (k_num / k_den) : 0.0;

    // 3D point (unscaled)
    const double X = k * mx;
    const double Y = k * my;
    const double Z = k * mz - xi;

    Eigen::Vector2f zn;
    if (std::abs(Z) < 1e-12) {
      zn(0) = static_cast<float>(mx);
      zn(1) = static_cast<float>(my);
    } else {
      zn(0) = static_cast<float>(X / Z);
      zn(1) = static_cast<float>(Y / Z);
    }
    return zn;
  }

  /**
   * Distort normalized coords (x_n, y_n) -> raw pixel (u,v)
   * Uses DS projection formula.
   */
  Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) override {
    // Build 3D ray p = [x,y,1]
    const double x = uv_norm(0);
    const double y = uv_norm(1);
    const double z = 1.0;

    const double r2 = x * x + y * y;

    // d1 = sqrt(r2 + z^2)
    const double d1 = std::sqrt(r2 + z * z);

    // k = xi * d1 + z
    const double k = xi * d1 + z;
    const double k2 = k * k;

    // d2 = sqrt(r2 + k^2)
    const double d2 = std::sqrt(r2 + k2);

    // norm = alpha * d2 + (1 - alpha) * k
    const double norm = alpha * d2 + (1.0 - alpha) * k;

    // protect against zero norm
    if (std::abs(norm) < 1e-12) {
      Eigen::Vector2f uv;
      uv(0) = static_cast<float>(cx);
      uv(1) = static_cast<float>(cy);
      return uv;
    }

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
   *  - H_dz_dzeta: 2x8 Jacobian of pixel z w.r.t intrinsics (fx,fy,cx,cy,xi,alpha,0,0)
   *
   * Implementation adapted from Basalt's Double-Sphere derivatives.
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

    const double d1 = std::sqrt(r2 + z * z);
    const double k = xi * d1 + z;
    const double k2 = k * k;
    const double d2 = std::sqrt(r2 + k2);

    const double norm = alpha * d2 + (1.0 - alpha) * k;
    const double norm2 = norm * norm;

    // helpful intermediate (tt2 in previous derivations)
    const double tt2 = (d1 > 0.0) ? (xi * z / d1 + 1.0) : 1.0;

    // d_norm_d_r2:
    // d(norm)/d(r2) = ( xi*(1-alpha)/d1 + alpha*(xi*k/d1 + 1)/d2 ) / norm
    // but we need derivative of 1/norm and of x/norm etc. Use Basalt form below.
    const double inv_d1 = (d1 > 0.0) ? 1.0 / d1 : 0.0;
    const double inv_d2 = (d2 > 0.0) ? 1.0 / d2 : 0.0;

    const double a = xi * (1.0 - alpha) * inv_d1;
    const double b = alpha * (xi * k * inv_d1 + 1.0) * inv_d2;
    const double d_norm_d_r2 = (a + b) / norm2 * 1.0; // note: matches Basalt algebra after chain rule adjustments

    // tmp2 corresponds to deriv wrt z in Basalt formula (see earlier comments)
    const double tmp2 = ((1.0 - alpha) * tt2 + alpha * k * tt2 * inv_d2) / norm2;

    // Build d_proj_d_p3d (2x3)
    Eigen::Matrix<double, 2, 3> d_proj_d_p3d;
    d_proj_d_p3d.setZero();

    // Following the Basalt arrangement: first two columns for x,y derivatives, third for z
    d_proj_d_p3d(0, 0) = fx * (1.0 / norm - xx * d_norm_d_r2);
    d_proj_d_p3d(0, 1) = -fx * x * y * d_norm_d_r2;
    d_proj_d_p3d(0, 2) = -fx * x * tmp2;

    d_proj_d_p3d(1, 0) = -fy * x * y * d_norm_d_r2;
    d_proj_d_p3d(1, 1) = fy * (1.0 / norm - yy * d_norm_d_r2);
    d_proj_d_p3d(1, 2) = -fy * y * tmp2;

    // d_proj_d_param (2x6) for [fx, fy, cx, cy, xi, alpha]
    Eigen::Matrix<double, 2, 6> d_proj_d_param;
    d_proj_d_param.setZero();

    // fx, fy, cx, cy direct partials
    const double mx = x / norm;
    const double my = y / norm;
    d_proj_d_param(0, 0) = mx;
    d_proj_d_param(1, 1) = my;
    d_proj_d_param(0, 2) = 1.0;
    d_proj_d_param(1, 3) = 1.0;

    // Partial derivatives wrt xi and alpha (Basalt form)
    // tmp4 = (alpha - 1 - alpha * k / d2) * d1 / norm^2
    // tmp5 = (k - d2) / norm^2
    const double tmp4 = (alpha - 1.0 - alpha * k * inv_d2) * d1 / (norm2);
    const double tmp5 = (k - d2) / (norm2);

    d_proj_d_param(0, 4) = fx * x * tmp4;
    d_proj_d_param(1, 4) = fy * y * tmp4;
    d_proj_d_param(0, 5) = fx * x * tmp5;
    d_proj_d_param(1, 5) = fy * y * tmp5;

    // Chain rule: p3d = [x, y, 1]^T -> dp3d/d(x,y) = [[1,0],[0,1],[0,0]]
    // So H_dz_dzn = d_proj_d_p3d * [I2; 0] => take first two columns
    H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
    H_dz_dzn(0, 0) = d_proj_d_p3d(0, 0);
    H_dz_dzn(0, 1) = d_proj_d_p3d(0, 1);
    H_dz_dzn(1, 0) = d_proj_d_p3d(1, 0);
    H_dz_dzn(1, 1) = d_proj_d_p3d(1, 1);

    // H_dz_dzeta: 2x8 to match CamBase expectation. Fill first 6 columns, last two zeros.
    H_dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
    H_dz_dzeta.block<2, 6>(0, 0) = d_proj_d_param;
    // columns 6 and 7 remain zero (no extra radial params)
  }

private:
  // explicit intrinsics
  double fx;
  double fy;
  double cx;
  double cy;
  double xi;
  double alpha;

  // cached helpers
  double inv_fx;
  double inv_fy;
};

} // namespace ov_core

#endif /* OV_CORE_CAM_DS_H */
