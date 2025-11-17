#ifndef OV_CORE_CAM_OMNI_RADTAN_H
#define OV_CORE_CAM_OMNI_RADTAN_H

#include "CamBase.h"
#include <cmath>
#include <cassert>
#include <iostream>
#include <sstream>
#include <limits>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace ov_core {

/**
 * @brief Omni + Radial-Tangential (radtan) camera model (header-only)
 *
 * Intrinsic layout (calib) expected: [ fu, fv, cu, cv, k1, k2, p1, p2, xi, alpha(not used)]^T
 *
 * Projection pipeline (distort_f):
 *   uv_norm (x_n,y_n)  --(unified/omni proj)--> (m_x,m_y)
 *                      --(radtan distortion)--> (m_x_d,m_y_d)
 *                      --(intrinsics)---------> pixel (u,v)
 *
 * Undistort (pixel -> uv_norm) is implemented iteratively using Gauss-Newton
 * with numerically computed Jacobians (finite differences).
 *
 * This variant captures the *first valid* calibration passed to set_value()
 * and uses it as the last-known-good baseline. The baseline is *sticky*:
 * once the first valid calibration is received, it will not be overwritten.
 *
 * Defensive checks avoid committing NaNs into the cached scalars used by the
 * projection code.
 */
class CamOmniRadtan : public CamBase {
public:
  CamOmniRadtan(int width, int height) : CamBase(width, height) {
    fx = fy = cx = cy = k1 = k2 = p1 = p2 = xi = 0.0;
    inv_fx = inv_fy = 0.0;
    camera_values = Eigen::VectorXd::Zero(10);
  }

  ~CamOmniRadtan() override {}

  // -----------------------
  // Helpers / Sentinels
  // -----------------------
  static inline Eigen::Vector2f invalid_sentinel() {
    // distinct, out-of-image / impossible value
    constexpr float kInvalid = -10000.0f;
    return Eigen::Vector2f(kInvalid, kInvalid);
  }

  static inline bool is_invalid_sentinel(const Eigen::Vector2f &v) {
    constexpr float kInvalid = -10000.0f;
    return v(0) == kInvalid && v(1) == kInvalid;
  }

  /**
   * Set camera intrinsics (calib vector layout follows header comment).
   *
   * This implementation captures the first valid calibration passed to this
   * function and stores it as the sticky last-known-good baseline. Any future
   * incoming calibration that contains non-finite values will be ignored and
   * replaced by the baseline.
   */
  virtual void set_value(const Eigen::MatrixXd &calib) override {
    // Assert we are of size ten: fx, fy, cx, cy, k1, k2, k3/p1, k4/p2, xi, alpha
    assert(calib.rows() == 10);
    camera_values = calib;

    // Cache intrinsics in convenient scalars
    fx = camera_values(0);
    fy = camera_values(1);
    cx = camera_values(2);
    cy = camera_values(3);
    k1 = camera_values(4);
    k2 = camera_values(5);
    p1 = camera_values(6);
    p2 = camera_values(7);
    xi = camera_values(8);

    _inv_K11 = 1.0 / fx;
    _inv_K13 = -cx / fx;
    _inv_K22 = 1.0 / fy;
    _inv_K23 = -cy / fy;

    camera_k_OPENCV = cv::Matx33d(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    camera_d_OPENCV = cv::Vec4d(0.0, 0.0, 0.0, 0.0);

    // safe print
    std::ostringstream out;
    out << "\033[32mCamOmniRadtan intrinsics:"
        << " fx=" << fx
        << " fy=" << fy
        << " cx=" << cx
        << " cy=" << cy
        << " k1=" << k1
        << " k2=" << k2
        << " p1=" << p1
        << " p2=" << p2
        << " xi=" << xi
        << "\033[0m" << std::endl;
    std::cout << out.str();
  }

  /**
   * Distort normalized coords (x_n, y_n) -> raw pixel (u,v)
   *
   * Steps:
   *  1) Build ray p = [x,y,1]^T
   *  2) d = sqrt(x^2 + y^2 + 1)
   *  3) m = [ x/(xi*d + 1), y/(xi*d + 1) ]
   *  4) apply radial-tangential distortion to m -> m_d
   *  5) pixel = [ fx * m_dx + cx, fy * m_dy + cy ]
   */
  Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) override {
    const double mx_u = static_cast<double>(uv_norm.x());
    const double my_u = static_cast<double>(uv_norm.y());

    double dx_u = 0.0, dy_u = 0.0;
    distortion(mx_u, my_u, &dx_u, &dy_u);

    const double mx_d = mx_u + dx_u;
    const double my_d = my_u + dy_u;

    const double u = fx * mx_d + cx;
    const double v = fy * my_d + cy;

    return Eigen::Vector2f(static_cast<float>(u), static_cast<float>(v));
  }

  void distortion(double mx_u, double my_u, double *dx_u,double *dy_u) const {
    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = mx_u * mx_u;
    my2_u = my_u * my_u;
    mxy_u = mx_u * my_u;
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    *dx_u = mx_u * rad_dist_u + 2 * p1 * mxy_u + p2 * (rho2_u + 2 * mx2_u);
    *dy_u = my_u * rad_dist_u + 2 * p2 * mxy_u + p1 * (rho2_u + 2 * my2_u);
  }

  void distortion(double mx_u, double my_u, double *dx_u,
                  double *dy_u, double *dxdmx, double *dydmx,
                  double *dxdmy, double *dydmy) const {
    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = mx_u * mx_u;
    my2_u = my_u * my_u;
    mxy_u = mx_u * my_u;
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    *dx_u = mx_u * rad_dist_u + 2 * p1 * mxy_u + p2 * (rho2_u + 2 * mx2_u);
    *dy_u = my_u * rad_dist_u + 2 * p2 * mxy_u + p1 * (rho2_u + 2 * my2_u);

    *dxdmx = 1 + rad_dist_u + k1 * 2 * mx2_u + k2 * rho2_u * 4 * mx2_u
        + 2 * p1 * my_u + 6 * p2 * mx_u;
    *dydmx = k1 * 2 * mx_u * my_u + k2 * 4 * rho2_u * mx_u * my_u
        + p1 * 2 * mx_u + 2 * p2 * my_u;
    *dxdmy = *dydmx;
    *dydmy = 1 + rad_dist_u + k1 * 2 * my2_u + k2 * rho2_u * 4 * my2_u
        + 6 * p1 * my_u + 2 * p2 * mx_u;
  }

  void undistortGN(double u_d, double v_d, double * u, double * v) const {
    *u = u_d;
    *v = v_d;

    double ubar = u_d;
    double vbar = v_d;
    const int n = 30;
    Eigen::Matrix2d F;

    double hat_u_d;
    double hat_v_d;

    // void OmniCameraGeometry::distortion(double mx_u, double my_u, 
    // 					  double *dx_u, double *dy_u,
    // 					  double *dxdmx, double *dydmx,
    // 					  double *dxdmy, double *dydmy) const
    for (int i = 0; i < n; i++) {
      distortion(ubar, vbar, &hat_u_d, &hat_v_d, &F(0, 0), &F(1, 0), &F(0, 1),
                &F(1, 1));

      Eigen::Vector2d e(u_d - ubar - hat_u_d, v_d - vbar - hat_v_d);
      Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;

      ubar += du[0];
      vbar += du[1];

      if (e.dot(e) < 1e-15)
        break;

    }
    *u = ubar;
    *v = vbar;
  }

  /**
   * Undistort raw pixel (u,v) -> normalized coordinates (x_n, y_n)
   *
   * Uses iterative Gauss-Newton solving with numerical Jacobian.
   * Returns the sentinel on failure.
   */
  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) override {
    // Read pixel (distorted) coordinates
    const double u_pix = static_cast<double>(uv_dist.x());
    const double v_pix = static_cast<double>(uv_dist.y());

    // Map pixel -> distorted normalized coordinates (same as used in lift_*):
    // mx_d = (u - u0) / gamma1  <==> _inv_K11 * u + _inv_K13
    // my_d = (v - v0) / gamma2  <==> _inv_K22 * v + _inv_K23
    const double mx_d = _inv_K11 * u_pix + _inv_K13;
    const double my_d = _inv_K22 * v_pix + _inv_K23;

    // Undistort using Gauss-Newton (fills mx_u,my_u)
    double mx_u = 0.0, my_u = 0.0;
    undistortGN(mx_d, my_d, &mx_u, &my_u);

    Eigen::Vector2f vec(static_cast<float>(mx_u), static_cast<float>(my_u));
    if (std::abs(vec(0)) > 500.1 || std::abs(vec(1)) > 500.1) {
        std::cout << "\033[31m" << "vec = " << vec.transpose() << "\033[0m" << std::endl;
    } else {
        std::cout << "vec = " << vec.transpose() << std::endl;
    }

    // Return as float Vector2f (cast from double)
    return Eigen::Vector2f(static_cast<float>(mx_u), static_cast<float>(my_u));
  }

  /**
   * Compute analytic (or numeric) Jacobians:
   *  - H_dz_dzn : 2x2 Jacobian of pixel w.r.t normalized coords [x_n, y_n]
   *  - H_dz_dzeta: 2xNparams Jacobian of pixel w.r.t intrinsics
   *
   * This implementation uses finite differences for safety and compactness.
   */
  void compute_distort_jacobian(const Eigen::Vector2d &uv_norm,
                                Eigen::MatrixXd &H_dz_dzn,
                                Eigen::MatrixXd &H_dz_dzeta) override {
    // Finite difference step sizes
    const double eps_coord = 1e-7;   // step for normalized coordinates
    const double eps_param = 1e-7;   // relative step for intrinsics

    // Number of intrinsic params (camera_values is expected to be size >= 1)
    const int Nparams = static_cast<int>(camera_values.size());
    if (Nparams <= 0) {
      H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
      H_dz_dzeta = Eigen::MatrixXd::Zero(2, 0);
      return;
    }

    // Ensure output sizes
    H_dz_dzn.resize(2, 2);
    H_dz_dzeta.resize(2, Nparams);

    // Convert uv_norm to floats for distort_f (which returns Vector2f)
    const Eigen::Vector2f uv_norm_f(static_cast<float>(uv_norm.x()),
                                    static_cast<float>(uv_norm.y()));

    // --- Base (unperturbed) pixel position ---
    const Eigen::Vector2f base_pix_f = distort_f(uv_norm_f);
    const Eigen::Vector2d base_pix(static_cast<double>(base_pix_f.x()),
                                  static_cast<double>(base_pix_f.y()));

    // --- Jacobian wrt normalized coordinates (2x2) using central differences ---
    for (int i = 0; i < 2; ++i) {
      Eigen::Vector2d uv_p_plus = uv_norm;
      Eigen::Vector2d uv_p_minus = uv_norm;

      uv_p_plus(i) += eps_coord;   // use parenthesis accessor
      uv_p_minus(i) -= eps_coord;

      const Eigen::Vector2f pv_f =
          distort_f(Eigen::Vector2f(static_cast<float>(uv_p_plus.x()),
                                    static_cast<float>(uv_p_plus.y())));
      const Eigen::Vector2f mv_f =
          distort_f(Eigen::Vector2f(static_cast<float>(uv_p_minus.x()),
                                    static_cast<float>(uv_p_minus.y())));

      const Eigen::Vector2d pv(static_cast<double>(pv_f.x()),
                              static_cast<double>(pv_f.y()));
      const Eigen::Vector2d mv(static_cast<double>(mv_f.x()),
                              static_cast<double>(mv_f.y()));

      Eigen::Vector2d col = (pv - mv) / (2.0 * eps_coord);
      H_dz_dzn.col(i) = col;
    }

    // --- Jacobian wrt intrinsics (2 x Nparams) via central finite differences ---
    // Save original camera values and cached intrinsics
    const Eigen::VectorXd saved_camera_values = camera_values;
    const double tiny = 1e-12;

    // Helper lambda to update cached scalar intrinsics from camera_values
    auto restore_cached_from_camera_values = [&]() {
      fx = camera_values(0);
      fy = camera_values(1);
      cx = camera_values(2);
      cy = camera_values(3);
      k1 = camera_values(4);
      k2 = camera_values(5);
      p1 = camera_values(6);
      p2 = camera_values(7);
      xi = camera_values(8);
      // If you have an alpha or extra param in camera_values(9) you may ignore or use it.
      // Update cached inv K helpers (same as updateTemporaries())
      _inv_K11 = 1.0 / fx;
      _inv_K13 = -cx / fx;
      _inv_K22 = 1.0 / fy;
      _inv_K23 = -cy / fy;
    };

    // Ensure we start with original cached values
    restore_cached_from_camera_values();

    for (int j = 0; j < Nparams; ++j) {
      const double orig = saved_camera_values(j);
      double delta = eps_param * std::max(1.0, std::abs(orig));
      if (delta == 0.0) delta = eps_param;

      // +delta
      camera_values(j) = orig + delta;
      restore_cached_from_camera_values();
      const Eigen::Vector2f pix_plus_f = distort_f(uv_norm_f);
      const Eigen::Vector2d pix_plus(static_cast<double>(pix_plus_f.x()),
                                    static_cast<double>(pix_plus_f.y()));

      // -delta
      camera_values(j) = orig - delta;
      restore_cached_from_camera_values();
      const Eigen::Vector2f pix_minus_f = distort_f(uv_norm_f);
      const Eigen::Vector2d pix_minus(static_cast<double>(pix_minus_f.x()),
                                      static_cast<double>(pix_minus_f.y()));

      // central finite difference
      H_dz_dzeta.col(j) = (pix_plus - pix_minus) / (2.0 * delta);

      // restore original value for next iteration
      camera_values(j) = orig;
      restore_cached_from_camera_values();
    }

    // Finally restore the saved camera values to be safe
    camera_values = saved_camera_values;
    restore_cached_from_camera_values();
  }

private:
  // explicit intrinsics
  double fx;
  double fy;
  double cx;
  double cy;
  double k1;
  double k2;
  double p1;
  double p2;
  double xi;

  // cached helpers
  double inv_fx;
  double inv_fy;

  double _inv_K11, _inv_K13, _inv_K22, _inv_K23;
};

} // namespace ov_core

#endif /* OV_CORE_CAM_OMNI_RADTAN_H */
