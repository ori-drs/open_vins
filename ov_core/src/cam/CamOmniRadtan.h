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
    // last_known_good_calib() remains uninitialized until first valid set_value call
    last_known_good_calib_initialized() = false;
  }

  ~CamOmniRadtan() override {}

  // -----------------------
  // Helpers / Sentinels
  // -----------------------
  static inline Eigen::Vector2f invalid_sentinel() {
    // distinct, out-of-image / impossible value
    constexpr float kInvalid = -1.0f;
    return Eigen::Vector2f(kInvalid, kInvalid);
  }

  static inline bool is_invalid_sentinel(const Eigen::Vector2f &v) {
    constexpr float kInvalid = -1.0f;
    return v(0) == kInvalid && v(1) == kInvalid;
  }

  static inline bool allFiniteParams(const Eigen::VectorXd &v) {
    for (int i = 0; i < v.size(); ++i) {
      if (!std::isfinite(v(i))) return false;
    }
    return true;
  }

  static inline std::string safe_print_double(double d) {
    if (!std::isfinite(d)) return std::string("INVALID");
    std::ostringstream ss; ss << d; return ss.str();
  }

  // last-known-good calibration (persisted across set_value calls)
  // stored but only set once (first valid calibration)
  static Eigen::VectorXd &last_known_good_calib() {
    static Eigen::VectorXd last = Eigen::VectorXd::Zero(10);
    return last;
  }

  static bool &last_known_good_calib_initialized() {
    static bool inited = false;
    return inited;
  }

  // -----------------------
  // Safe cached updater
  // -----------------------
  void update_cached_safe() {
    // only copy if finite, otherwise keep previous value
    if (camera_values.rows() >= 1 && std::isfinite(camera_values(0))) fx = camera_values(0);
    if (camera_values.rows() >= 2 && std::isfinite(camera_values(1))) fy = camera_values(1);
    if (camera_values.rows() >= 3 && std::isfinite(camera_values(2))) cx = camera_values(2);
    if (camera_values.rows() >= 4 && std::isfinite(camera_values(3))) cy = camera_values(3);
    if (camera_values.rows() >= 5 && std::isfinite(camera_values(4))) k1 = camera_values(4);
    if (camera_values.rows() >= 6 && std::isfinite(camera_values(5))) k2 = camera_values(5);
    if (camera_values.rows() >= 7 && std::isfinite(camera_values(6))) p1 = camera_values(6);
    if (camera_values.rows() >= 8 && std::isfinite(camera_values(7))) p2 = camera_values(7);
    if (camera_values.rows() >= 9 && std::isfinite(camera_values(8))) xi = camera_values(8);

    // compute cached inverses only if finite and non-zero
    if (std::isfinite(fx) && fx != 0.0) inv_fx = 1.0 / fx;
    else inv_fx = 0.0;

    if (std::isfinite(fy) && fy != 0.0) inv_fy = 1.0 / fy;
    else inv_fy = 0.0;

    // OpenCV helpers guarded
    if (std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) && std::isfinite(cy)) {
      camera_k_OPENCV = cv::Matx33d(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    }
    if (std::isfinite(k1) && std::isfinite(k2) && std::isfinite(p1) && std::isfinite(p2)) {
      camera_d_OPENCV = cv::Vec4d(k1, k2, p1, p2);
    }
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
    assert(calib.rows() == 10);
    Eigen::VectorXd cand = calib; // copy

    if (!allFiniteParams(cand)) {
      // incoming calib contains NaNs
      if (!last_known_good_calib_initialized()) {
        // We haven't seen a single valid calibration yet -> cannot accept NaN as first value.
        std::cerr << "[CamOmniRadtan] set_value(): incoming calibration contains non-finite values "
                     "and no valid calibration has been seen yet — ignoring.\n";
        // camera_values left unchanged; wait for first valid set_value()
      } else {
        // We have a baseline: ignore the NaN update and keep baseline values
        std::cerr << "[CamOmniRadtan] set_value(): incoming calibration contains non-finite values — keeping first-known-good baseline.\n";
        camera_values = last_known_good_calib();
      }
    } else {
      // candidate is valid
      camera_values = cand;
      if (!last_known_good_calib_initialized()) {
        // this is the first valid calibration we've seen -> capture it as the baseline (sticky)
        last_known_good_calib() = cand;
        last_known_good_calib_initialized() = true;
        std::cerr << "[CamOmniRadtan] set_value(): captured first valid calibration as last-known-good baseline.\n";
      } else {
        // baseline is sticky: do NOT overwrite it. We still update camera_values to the new valid set
        // camera_values already set to cand above
      }
    }

    // (Re)compute cached scalars safely
    update_cached_safe();

    // safe print
    std::ostringstream out;
    out << "\033[32mCamOmniRadtan intrinsics:"
        << " fx=" << safe_print_double(fx)
        << " fy=" << safe_print_double(fy)
        << " cx=" << safe_print_double(cx)
        << " cy=" << safe_print_double(cy)
        << " k1=" << safe_print_double(k1)
        << " k2=" << safe_print_double(k2)
        << " p1=" << safe_print_double(p1)
        << " p2=" << safe_print_double(p2)
        << " xi=" << safe_print_double(xi)
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
    const double x = static_cast<double>(uv_norm(0));
    const double y = static_cast<double>(uv_norm(1));
    const double z = 1.0;

    // unified / omni projection
    const double d = std::sqrt(x * x + y * y + z * z);
    const double denom = xi * d + z; // note: z == 1.0
    if (!std::isfinite(d) || !std::isfinite(denom) || std::abs(denom) < 1e-14) {
      return invalid_sentinel();
    }

    const double m_x = x / denom;
    const double m_y = y / denom;

    // radial-tangential distortion (Brown-Conrady / OpenCV radtan)
    const double r2 = m_x * m_x + m_y * m_y;
    const double radial = 1.0 + k1 * r2 + k2 * r2 * r2;

    // tangential terms
    const double tang_x = 2.0 * p1 * m_x * m_y + p2 * (r2 + 2.0 * m_x * m_x);
    const double tang_y = p1 * (r2 + 2.0 * m_y * m_y) + 2.0 * p2 * m_x * m_y;

    // distorted m
    const double mxd = m_x * radial + tang_x;
    const double myd = m_y * radial + tang_y;

    // pixel coordinates
    Eigen::Vector2f uv;
    uv(0) = static_cast<float>(fx * mxd + cx);
    uv(1) = static_cast<float>(fy * myd + cy);

    if (!std::isfinite(uv(0)) || !std::isfinite(uv(1))) return invalid_sentinel();

    return uv;
  }

  /**
   * Undistort raw pixel (u,v) -> normalized coordinates (x_n, y_n)
   *
   * Uses iterative Gauss-Newton solving with numerical Jacobian.
   * Returns the sentinel on failure.
   */
  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) override {
    // observed pixel
    const double u_obs = static_cast<double>(uv_dist(0));
    const double v_obs = static_cast<double>(uv_dist(1));

    // initial guess: use the (simple) inverse intrinsics -> this is a guess for m_dist (post-distortion)
    Eigen::Vector2d m; // this represents the "m" coordinates used by distortion (m_x, m_y)
    m(0) = (u_obs - cx) * inv_fx; // approximate m_x
    m(1) = (v_obs - cy) * inv_fy; // approximate m_y

    const int max_iters = 100;
    const double tol_res = 1e-9;     // stop on residual
    const double tol_delta = 1e-14;  // stop on parameter change
    const double fd_eps = 1e-8;      // finite-diff step for numerical Jacobian

    for (int iter = 0; iter < max_iters; ++iter) {
      // Given candidate m, compute predicted pixel using distort_f-like pipeline
      const double m_x = m(0);
      const double m_y = m(1);
      const double r2 = m_x * m_x + m_y * m_y;
      const double radial = 1.0 + k1 * r2 + k2 * r2 * r2;
      const double tang_x = 2.0 * p1 * m_x * m_y + p2 * (r2 + 2.0 * m_x * m_x);
      const double tang_y = p1 * (r2 + 2.0 * m_y * m_y) + 2.0 * p2 * m_x * m_y;
      const double mxd = m_x * radial + tang_x;
      const double myd = m_y * radial + tang_y;
      const double u_pred = fx * mxd + cx;
      const double v_pred = fy * myd + cy;

      // residual (predicted - observed)
      Eigen::Vector2d r;
      r(0) = u_pred - u_obs;
      r(1) = v_pred - v_obs;
      const double res_norm = std::sqrt(r(0) * r(0) + r(1) * r(1));
      if (!std::isfinite(res_norm)) return invalid_sentinel();
      if (res_norm < tol_res) break;

      // numerical Jacobian J = d(uv_pred) / d(m)  -> 2x2
      Eigen::Matrix2d J;
      for (int dim = 0; dim < 2; ++dim) {
        Eigen::Vector2d m_eps = m;
        m_eps(dim) += fd_eps;

        const double m_x_e = m_eps(0);
        const double m_y_e = m_eps(1);
        const double r2_e = m_x_e * m_x_e + m_y_e * m_y_e;
        const double radial_e = 1.0 + k1 * r2_e + k2 * r2_e * r2_e;
        const double tang_x_e = 2.0 * p1 * m_x_e * m_y_e + p2 * (r2_e + 2.0 * m_x_e * m_x_e);
        const double tang_y_e = p1 * (r2_e + 2.0 * m_y_e * m_y_e) + 2.0 * p2 * m_x_e * m_y_e;
        const double mxd_e = m_x_e * radial_e + tang_x_e;
        const double myd_e = m_y_e * radial_e + tang_y_e;
        const double u_pred_e = fx * mxd_e + cx;
        const double v_pred_e = fy * myd_e + cy;

        J(0, dim) = (u_pred_e - u_pred) / fd_eps;
        J(1, dim) = (v_pred_e - v_pred) / fd_eps;
      }

      // Solve for delta: J * delta = r   -> we will take uv_new = m - delta (reduce predicted toward observed)
      // Use damped normal equations (Levenberg-like) for robustness
      Eigen::Matrix2d JTJ = J.transpose() * J;
      const double lambda = 1e-6; // small damping
      JTJ.diagonal().array() += lambda;
      Eigen::Vector2d rhs = J.transpose() * r;

      // solve JTJ * delta = rhs
      Eigen::FullPivLU<Eigen::Matrix2d> lu(JTJ);
      if (!lu.isInvertible()) {
        // Try a fallback solve using pseudo-inverse from J (more robust in degenerate cases)
        Eigen::ColPivHouseholderQR<Eigen::Matrix2d> qr(J);
        if (qr.rank() == 0) return invalid_sentinel();
        Eigen::Vector2d delta = qr.solve(r);
        if (!std::isfinite(delta(0)) || !std::isfinite(delta(1))) return invalid_sentinel();
        m -= delta;
        if (delta.norm() < tol_delta) break;
        continue;
      }

      Eigen::Vector2d delta = lu.solve(rhs);
      if (!std::isfinite(delta(0)) || !std::isfinite(delta(1))) return invalid_sentinel();

      // apply update
      m -= delta;

      if (delta.norm() < tol_delta) break;
    }

    // final projection check (ensure result is finite and consistent)
    {
      const double m_x = m(0);
      const double m_y = m(1);
      const double r2 = m_x * m_x + m_y * m_y;
      const double radial = 1.0 + k1 * r2 + k2 * r2 * r2;
      const double tang_x = 2.0 * p1 * m_x * m_y + p2 * (r2 + 2.0 * m_x * m_x);
      const double tang_y = p1 * (r2 + 2.0 * m_y * m_y) + 2.0 * p2 * m_x * m_y;
      const double mxd = m_x * radial + tang_x;
      const double myd = m_y * radial + tang_y;
      const double u_pred = fx * mxd + cx;
      const double v_pred = fy * myd + cy;
      if (!std::isfinite(u_pred) || !std::isfinite(v_pred)) return invalid_sentinel();
    }

    // Solve for x,y from m numerically (small fixed-point iterations). This step is needed because
    // undistort_f should return normalized coords (x_n,y_n), not m.
    Eigen::Vector2d xy; // unknowns x,y
    // initial guess: assume xi small => x ~= m, y ~= m
    xy = m;
    const int max_back_iters = 30;
    const double tol_back = 1e-12;
    for (int it = 0; it < max_back_iters; ++it) {
      const double x = xy(0);
      const double y = xy(1);
      const double d = std::sqrt(x * x + y * y + 1.0);
      const double denom = xi * d + 1.0;
      if (!std::isfinite(denom) || std::abs(denom) < 1e-14) {
        return invalid_sentinel();
      }
      const double m_x_est = x / denom;
      const double m_y_est = y / denom;
      Eigen::Vector2d res;
      res(0) = m_x_est - m(0);
      res(1) = m_y_est - m(1);
      if (!std::isfinite(res(0)) || !std::isfinite(res(1))) return invalid_sentinel();
      if (res.norm() < 1e-12) break;

      // numerical Jacobian of m = f(x,y) -> 2x2
      Eigen::Matrix2d Jb;
      const double h = 1e-8;
      for (int dmm = 0; dmm < 2; ++dmm) {
        Eigen::Vector2d xy_eps = xy;
        xy_eps(dmm) += h;
        const double x_e = xy_eps(0);
        const double y_e = xy_eps(1);
        const double d_e = std::sqrt(x_e * x_e + y_e * y_e + 1.0);
        const double denom_e = xi * d_e + 1.0;
        if (!std::isfinite(denom_e) || std::abs(denom_e) < 1e-14) return invalid_sentinel();
        const double m_x_e = x_e / denom_e;
        const double m_y_e = y_e / denom_e;
        Jb(0, dmm) = (m_x_e - m_x_est) / h;
        Jb(1, dmm) = (m_y_e - m_y_est) / h;
      }

      // Solve Jb * delta_xy = res  (we want to correct xy to reduce res -> xy_new = xy - delta)
      Eigen::Matrix2d JtbJ = Jb.transpose() * Jb;
      JtbJ.diagonal().array() += 1e-9;
      Eigen::Vector2d rhs = Jb.transpose() * res;
      Eigen::FullPivLU<Eigen::Matrix2d> lu2(JtbJ);
      if (!lu2.isInvertible()) {
        // fallback to simple damped gradient step
        xy -= 0.1 * rhs;
      } else {
        Eigen::Vector2d delta_xy = lu2.solve(rhs);
        if (!std::isfinite(delta_xy(0)) || !std::isfinite(delta_xy(1))) return invalid_sentinel();
        xy -= delta_xy;
        if (delta_xy.norm() < tol_back) break;
      }
    }

    if (!std::isfinite(xy(0)) || !std::isfinite(xy(1))) return invalid_sentinel();
    return Eigen::Vector2f(static_cast<float>(xy(0)), static_cast<float>(xy(1)));
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
    // finite-diff base step (relative scaling applied per-parameter)
    const double h_rel = 1e-7;

    // compute base projection once
    Eigen::Vector2f base = distort_f(Eigen::Vector2f(static_cast<float>(uv_norm(0)),
                                                    static_cast<float>(uv_norm(1))));
    if (!std::isfinite(base(0)) || !std::isfinite(base(1))) {
      H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
      H_dz_dzeta = Eigen::MatrixXd::Zero(2, camera_values.rows());
      return;
    }

    // --------------------------
    // 1) Jacobian w.r.t. uv_norm (2x2) - numeric FD
    // --------------------------
    H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
    {
      const double h = 1e-7; // small absolute FD for uv_norm
      for (int dim = 0; dim < 2; ++dim) {
        Eigen::Vector2d pert = uv_norm;
        pert(dim) += h;
        Eigen::Vector2f proj_p = distort_f(Eigen::Vector2f(static_cast<float>(pert(0)),
                                                          static_cast<float>(pert(1))));
        // handle non-finite result defensively
        if (!std::isfinite(proj_p(0)) || !std::isfinite(proj_p(1))) {
          H_dz_dzn(0, dim) = 0.0;
          H_dz_dzn(1, dim) = 0.0;
        } else {
          H_dz_dzn(0, dim) = (static_cast<double>(proj_p(0)) - static_cast<double>(base(0))) / h;
          H_dz_dzn(1, dim) = (static_cast<double>(proj_p(1)) - static_cast<double>(base(1))) / h;
        }
      }
    }

    // --------------------------
    // 2) Jacobian w.r.t. intrinsics zeta (2 x Nparams) - numeric FD
    // --------------------------
    const int Nparams = static_cast<int>(camera_values.rows());
    H_dz_dzeta = Eigen::MatrixXd::Zero(2, Nparams);

    // cache original parameter vector (N x 1)
    Eigen::VectorXd orig = camera_values; // small copy, N typically small (<=10)

    // Use the safe updater defined on the object
    auto update_cached = [&]() {
      update_cached_safe();
    };

    // ensure cached values reflect orig (should already, but keep safe)
    update_cached();

    // temporary vector for projection
    Eigen::Vector2f proj_p;

    for (int i = 0; i < Nparams; ++i) {
      const double p0 = orig(i);
      // relative epsilon (protect against tiny or huge params)
      const double eps = h_rel * std::max(1.0, std::abs(p0));

      // perturb in-place (no full matrix copy)
      camera_values(i) = p0 + eps;

      // update cached scalars to reflect the perturbation
      update_cached();

      // project
      proj_p = distort_f(Eigen::Vector2f(static_cast<float>(uv_norm(0)),
                                        static_cast<float>(uv_norm(1))));

      if (!std::isfinite(proj_p(0)) || !std::isfinite(proj_p(1))) {
        // fallback: try smaller absolute step if the large relative step produced invalid projection
        const double eps_small = h_rel * 1e-3 * std::max(1.0, std::abs(p0));
        camera_values(i) = p0 + eps_small;
        update_cached();
        proj_p = distort_f(Eigen::Vector2f(static_cast<float>(uv_norm(0)),
                                          static_cast<float>(uv_norm(1))));
        // if still invalid, fill zeros and restore
        if (!std::isfinite(proj_p(0)) || !std::isfinite(proj_p(1))) {
          H_dz_dzeta(0, i) = 0.0;
          H_dz_dzeta(1, i) = 0.0;
          camera_values(i) = p0;       // restore
          update_cached();            // restore cached scalars
          continue;
        } else {
          // use eps_small as effective delta
          H_dz_dzeta(0, i) = (static_cast<double>(proj_p(0)) - static_cast<double>(base(0))) / eps_small;
          H_dz_dzeta(1, i) = (static_cast<double>(proj_p(1)) - static_cast<double>(base(1))) / eps_small;
          camera_values(i) = p0; // restore
          update_cached();
          continue;
        }
      }

      // normal case: finite projection
      H_dz_dzeta(0, i) = (static_cast<double>(proj_p(0)) - static_cast<double>(base(0))) / eps;
      H_dz_dzeta(1, i) = (static_cast<double>(proj_p(1)) - static_cast<double>(base(1))) / eps;

      // restore original parameter value in-place and update cached scalars
      camera_values(i) = p0;
      update_cached();
    }

    // final: ensure everything restored to original
    camera_values = orig;
    update_cached();
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
};

} // namespace ov_core

#endif /* OV_CORE_CAM_OMNI_RADTAN_H */
