#include "aiming.hpp"
#include "lessons.hpp"
#include <algorithm>
#include <cmath>
namespace training {
std::optional<BallisticSolution> solve_ballistic(double v, double d, double h, double g) {
  if (!std::isfinite(v) || !std::isfinite(d) || !std::isfinite(h) ||
      !std::isfinite(g) || v <= 0 || d <= 1e-6 || g <= 0) return {};
  const double a = g*d*d/(2*v*v), disc = d*d-4*a*(a+h);
  if (!std::isfinite(disc) || disc < 0) return {};
  // Rationalized low-arc root remains accurate at high speed.
  const double theta = std::atan(2*(a+h)/(d+std::sqrt(disc)));
  const double flight = d/(v*std::cos(theta));
  if (!std::isfinite(flight) || flight <= 0) return {};
  return BallisticSolution{theta,flight};
}
AimSolution solve_aim(const Target &t, TimePoint stamp, double speed,
                      const Eigen::Quaterniond &imu, const AimParameters &p, TimePoint now) {
  AimSolution s;
  s.observation_age_s=std::chrono::duration<double>(now-stamp).count();
  if (!t.position.allFinite() || !t.velocity.allFinite() ||
      !imu.coeffs().allFinite() || std::abs(imu.norm()-1)>.01 ||
      !std::isfinite(speed) || speed<=0 || !p.muzzle_gimbal.allFinite() ||
      !std::isfinite(p.delay_s) || p.delay_s<0 ||
      !std::isfinite(p.max_age_s) || p.max_age_s<=0 ||
      !std::isfinite(p.mount_elevation) || !std::isfinite(p.pitch_limit) ||
      p.pitch_limit<=0 || p.pitch_limit>kPi/2 ||
      !std::isfinite(p.convergence_s) || p.convergence_s<=0 ||
      p.max_iterations<=0 || p.max_iterations>100) {
    s.reason="invalid input";return s;
  }
  if (s.observation_age_s<-.005 || s.observation_age_s>p.max_age_s) {
    s.reason="stale/future observation";return s;
  }
  s.observation_age_s=std::max(0.,s.observation_age_s);
  const Eigen::Vector3d muzzle=imu.normalized()*p.muzzle_gimbal;
  double flight=0;bool converged=false;
  for (int i=0;i<p.max_iterations;++i) {
    s.iterations=i+1;s.lead_s=s.observation_age_s+p.delay_s+flight;
    s.intercept=t.position+t.velocity*s.lead_s;
    const Eigen::Vector3d delta=s.intercept-muzzle;
    const auto b=solve_ballistic(speed,std::hypot(delta.x(),delta.y()),delta.z(),p.gravity);
    if (!b) {s.reason="unreachable ballistic target";return s;}
    s.elevation=b->elevation;s.flight_s=b->flight_s;
    if (std::abs(s.flight_s-flight)<p.convergence_s) {converged=true;break;}
    flight=s.flight_s;
  }
  if (!converged) {s.reason="intercept did not converge";return s;}
  const Eigen::Vector3d delta=s.intercept-muzzle;
  s.command.yaw=std::atan2(delta.y(),delta.x());
  // IO negates pitch: wire pitch = 25deg - ballistic elevation.
  s.command.pitch=s.elevation-p.mount_elevation;
  if (std::abs(s.command.pitch)>p.pitch_limit) {s.reason="gimbal pitch limit";return s;}
  s.command.control=true;s.reason="ballistic solution";return s;
}
void FireControl::reset() {previous_.reset();reason="waiting for target";}
Command FireControl::update(const AimSolution &s, const Eigen::Quaterniond &q,
                           bool trusted, unsigned generation) {
  Command c=s.command;c.shoot=false;
  if (!c.control || !trusted || !q.coeffs().allFinite() || std::abs(q.norm()-1)>.01) {
    reset();reason=!c.control?s.reason:"unconfirmed/lost target";c.control=false;return c;
  }
  const auto R=q.normalized().toRotationMatrix();
  const double yaw=std::atan2(R(1,0),R(0,0));
  const double pitch=std::atan2(R(2,0),std::hypot(R(0,0),R(1,0)));
  yaw_error=std::abs(std::remainder(c.yaw-yaw,2*kPi));pitch_error=std::abs(c.pitch-pitch);
  if (previous_ && generation==generation_) {
    const double ey=std::abs(std::remainder(previous_->yaw-yaw,2*kPi));
    const double ep=std::abs(previous_->pitch-pitch);
    c.shoot=ey<.012 && ep<.007 && yaw_error<.018 && pitch_error<.009;
    reason=c.shoot?"FIRE: feedback aligned":"HOLD: gimbal converging";
  } else reason="HOLD: first command/new track";
  previous_=c;generation_=generation;return c;
}
Command lesson05_aiming(const Target &t, TimePoint stamp, double speed) {
  // Textbook compatibility assumes coincident muzzle/gimbal; the actual main
  // passes IMU and the rigid muzzle extrinsic. Firing always requires feedback.
  AimParameters p;p.muzzle_gimbal.setZero();
  return solve_aim(t,stamp,speed,Eigen::Quaterniond::Identity(),p).command;
}
} // namespace training
